#!/usr/bin/env bash

set -euo pipefail

# Groot AppImage install strategy:
# 1) Use GROOT_APPIMAGE_URL when explicitly set.
# 2) Otherwise, use an explicit release manifest (no blind URL guessing).
# 3) Try GROOT_VERSION first, then GROOT_FALLBACK_VERSIONS.
# 4) Optionally verify integrity with GROOT_SHA256.

GROOT_VERSION="${GROOT_VERSION:-1.9.0}"
GROOT_FALLBACK_VERSIONS="${GROOT_FALLBACK_VERSIONS:-1.8.1,1.8.0,1.7.1,1.7.0,1.6.1}"
GROOT_APPIMAGE_URL="${GROOT_APPIMAGE_URL:-}"
GROOT_SHA256="${GROOT_SHA256:-}"

ARCH="$(uname -m)"
case "${ARCH}" in
    x86_64|amd64)
        APPIMAGE_ARCH="x86_64"
        ;;
    aarch64|arm64)
        APPIMAGE_ARCH="aarch64"
        ;;
    *)
        echo "ERROR: Unsupported architecture for Groot AppImage: ${ARCH}"
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

APP_DIR="$PROJECT_ROOT/apps/groot"
GROOT_APPIMAGE="$APP_DIR/groot.AppImage"

mkdir -p "$APP_DIR"

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

    if [ -n "${GROOT_SHA256}" ]; then
        local actual
        actual=$(sha256sum "${file}" | awk '{print $1}')
        if [ "${actual}" != "${GROOT_SHA256}" ]; then
            echo "SHA256 mismatch for Groot AppImage."
            return 1
        fi
    fi

    return 0
}

install_from_url() {
    local url="$1"
    local tmp_file
    tmp_file=$(mktemp "${APP_DIR}/groot.AppImage.tmp.XXXXXX")

    echo "Downloading Groot AppImage from: ${url}"
    if ! download_to_file "${url}" "${tmp_file}"; then
        rm -f "${tmp_file}"
        return 1
    fi

    if ! verify_file "${tmp_file}"; then
        rm -f "${tmp_file}"
        return 1
    fi

    mv -f "${tmp_file}" "${GROOT_APPIMAGE}"
    chmod +x "${GROOT_APPIMAGE}"
    return 0
}

install_by_version() {
    local version="$1"
    local key="${APPIMAGE_ARCH}:${version}"
    local url=""

    case "${key}" in
        # x86_64 historical transition: old S3 (<=1.6.1) -> new R2 CDN (>=1.7.0)
        x86_64:1.6.1)
            url="https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/Groot2-v1.6.1-x86_64.AppImage"
            ;;
        x86_64:1.7.0|x86_64:1.7.1|x86_64:1.8.0|x86_64:1.8.1|x86_64:1.9.0)
            url="https://pub-32cef6782a9e411e82222dee82af193e.r2.dev/Groot2-v${version}-x86_64.AppImage"
            ;;
        # aarch64 AppImage is available from 1.7.1 onwards on R2 CDN
        aarch64:1.7.1|aarch64:1.8.0|aarch64:1.8.1|aarch64:1.9.0)
            url="https://pub-32cef6782a9e411e82222dee82af193e.r2.dev/Groot2-v${version}-aarch64.AppImage"
            ;;
        *)
            echo "Skip unsupported/unpublished release for ${APPIMAGE_ARCH}: ${version}"
            return 1
            ;;
    esac

    if install_from_url "${url}"; then
        echo "Installed Groot version ${version} (${APPIMAGE_ARCH})."
        return 0
    fi
    return 1
}

if [ -x "${GROOT_APPIMAGE}" ]; then
    echo "Groot AppImage already exists: ${GROOT_APPIMAGE}"
    exit 0
fi

if [ -n "${GROOT_APPIMAGE_URL}" ]; then
    if install_from_url "${GROOT_APPIMAGE_URL}"; then
        exit 0
    fi
    echo "Pinned GROOT_APPIMAGE_URL failed: ${GROOT_APPIMAGE_URL}"
fi

versions=("${GROOT_VERSION}")
fallback_versions="${GROOT_FALLBACK_VERSIONS//,/ }"
for fallback in ${fallback_versions}; do
    if [ "${fallback}" != "${GROOT_VERSION}" ]; then
        versions+=("${fallback}")
    fi
done

for version in "${versions[@]}"; do
    if install_by_version "${version}"; then
        exit 0
    fi
done

echo "ERROR: Unable to download Groot AppImage from all configured sources."
echo "Hint: set GROOT_APPIMAGE_URL to a valid direct AppImage URL."
exit 1
