#!/bin/bash -e

# Script to download the Groot AppImage
WEB_URL="https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer"
GROOT_VERSION="1.6.1"
GROOT_FILE="Groot2-v${GROOT_VERSION}-x86_64.AppImage"

WS_DIR=$HOME/workspace
GROOT_APPIMAGE=./groot.AppImage

cd $WS_DIR/groot

# Check if the AppImage is exist
if [ ! -f $GROOT_APPIMAGE ]; then
    echo "Downloading Groot AppImage: $GROOT_FILE"

    # Install the Groot AppImage
    wget  -O $GROOT_APPIMAGE --user-agent="Mozilla/5.0 (Windows NT 6.1; Trident/7.0; rv:11.0) like Gecko" \
        $WEB_URL/$GROOT_FILE
else
    echo "Groot AppImage already exist: $GROOT_APPIMAGE"
fi

sudo chmod +x $WS_DIR/groot/groot.AppImage
