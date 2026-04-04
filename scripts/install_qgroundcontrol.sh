#!/usr/bin/env bash

set -euo pipefail

# QGroundControl AppImage install strategy:
# 1) Use QGC_APPIMAGE_URL when explicitly set.
# 2) Otherwise, resolve by QGC_VERSION (latest or explicit release tag).
# 3) Optionally verify integrity with QGC_SHA256.

QGC_VERSION="${QGC_VERSION:-latest}"
QGC_APPIMAGE_URL="${QGC_APPIMAGE_URL:-}"
QGC_SHA256="${QGC_SHA256:-}"

ARCH="$(uname -m)"
case "${ARCH}" in
    x86_64|amd64)
        APPIMAGE_ARCH="x86_64"
        ;;
    aarch64|arm64)
        APPIMAGE_ARCH="aarch64"
        ;;
    *)
        echo "ERROR: Unsupported architecture for QGroundControl AppImage: ${ARCH}"
        exit 1
        ;;
esac

if [ -n "${ROS2X_ROOT:-}" ]; then
    PROJECT_ROOT="${ROS2X_ROOT}"
elif [ -n "${ROS2X_WS:-}" ] && [[ "${ROS2X_WS}" == */ros2_ws ]]; then
    PROJECT_ROOT="${ROS2X_WS%/ros2_ws}"
else
    PROJECT_ROOT="$HOME/ROS2X"
fi

APP_DIR="${PROJECT_ROOT}/apps/qgroundcontrol"
QGC_APPIMAGE="${APP_DIR}/qgroundcontrol.AppImage"

mkdir -p "${APP_DIR}" "${APP_DIR}/config" "${APP_DIR}/data"

download_to_file() {
    local url="$1"
    local out_file="$2"
    local ua="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko)"

    if command -v curl >/dev/null 2>&1; then
        curl -fL --retry 3 --connect-timeout 20 -A "${ua}" -o "${out_file}" "${url}"
    else
        wget --tries=3 --timeout=20 --user-agent="${ua}" -O "${out_file}" "${url}"
    fi
}

verify_file() {
    local file="$1"
    if [ ! -s "${file}" ]; then
        echo "Download failed: empty file."
        return 1
    fi

    # AppImage should not be tiny; reject obvious error pages.
    local bytes
    bytes=$(wc -c < "${file}")
    if [ "${bytes}" -lt 50000000 ]; then
        echo "Downloaded file is unexpectedly small (${bytes} bytes)."
        return 1
    fi

    if [ -n "${QGC_SHA256}" ]; then
        local actual
        actual=$(sha256sum "${file}" | awk '{print $1}')
        if [ "${actual}" != "${QGC_SHA256}" ]; then
            echo "SHA256 mismatch for QGroundControl AppImage."
            return 1
        fi
    fi

    return 0
}

install_from_url() {
    local url="$1"
    local tmp_file
    tmp_file=$(mktemp "${APP_DIR}/qgroundcontrol.AppImage.tmp.XXXXXX")

    echo "Downloading QGroundControl AppImage from: ${url}"
    if ! download_to_file "${url}" "${tmp_file}"; then
        rm -f "${tmp_file}"
        return 1
    fi

    if ! verify_file "${tmp_file}"; then
        rm -f "${tmp_file}"
        return 1
    fi

    mv -f "${tmp_file}" "${QGC_APPIMAGE}"
    chmod +x "${QGC_APPIMAGE}"
    return 0
}

install_by_version() {
    local version_raw="$1"
    local version="${version_raw#v}"

    if [ "${APPIMAGE_ARCH}" != "x86_64" ]; then
        echo "ERROR: Official QGroundControl AppImage currently provides x86_64 builds only."
        echo "Hint: set QGC_APPIMAGE_URL to your own ${APPIMAGE_ARCH} build URL."
        return 1
    fi

    local urls=()
    if [ "${version}" = "latest" ]; then
        urls+=("https://github.com/mavlink/qgroundcontrol/releases/latest/download/QGroundControl-x86_64.AppImage")
        urls+=("https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage")
    else
        urls+=("https://github.com/mavlink/qgroundcontrol/releases/download/v${version}/QGroundControl-x86_64.AppImage")
    fi

    local url
    for url in "${urls[@]}"; do
        if install_from_url "${url}"; then
            echo "Installed QGroundControl version ${version_raw} (${APPIMAGE_ARCH})."
            return 0
        fi
    done

    return 1
}

if [ -x "${QGC_APPIMAGE}" ]; then
    echo "QGroundControl AppImage already exists: ${QGC_APPIMAGE}"
    exit 0
fi

if [ -n "${QGC_APPIMAGE_URL}" ]; then
    if install_from_url "${QGC_APPIMAGE_URL}"; then
        exit 0
    fi
    echo "Pinned QGC_APPIMAGE_URL failed: ${QGC_APPIMAGE_URL}"
fi

if install_by_version "${QGC_VERSION}"; then
    exit 0
fi

echo "ERROR: Unable to download QGroundControl AppImage."
echo "Hint: set QGC_APPIMAGE_URL to a valid direct AppImage URL."
exit 1
