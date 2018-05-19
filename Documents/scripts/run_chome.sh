#!/bin/bash -x

FAKEUSER="${1:-fake-chrome-user}"
CHROMEROOT=$HOME/.chromeroot/

mkdir -p ${CHROMEROOT}

export PROFILE="${CHROMEROOT}/${FAKEUSER}-chromium-profile"
export DISK_CACHEDIR="${CHROMEROOT}/${FAKEUSER}-chromium-profile-cache"
export DISK_CACHESIZE=4096
export MEDIA_CACHESIZE=4096

PARANOID_OPTIONS="\
        --no-displaying-insecure-content \
        --no-referrers \
        --disable-zero-suggest \
        --disable-sync  \
        --cipher-suite-blacklist=0x0004,0x0005,0xc011,0xc007 \
        --enable-sandbox-logging >/dev/null 2>&1
        "


/opt/google/chrome/google-chrome \
        --remember-cert-error-decisions \
        --ignore-urlfetcher-cert-requests \
        --allow-running-insecure-content \
        --window-position=200,400 \
        --window-size=200,400 \
        --no-pings \
        --user-data-dir=${PROFILE} \
        --disk-cache-dir=${DISK_CACHEDIR} \
        --disk-cache-size=${DISK_CACHESIZE} \
        --media-cache-size=${MEDIA_CACHESIZE} \
        http://localhost:4567
        2>&1

#--ignore-certificate-errors \
#--proxy-server="socks4://localhost:30604" \
#--host-resolver-rules="MAP * 0.0.0.0 , EXCLUDE localhost" \

