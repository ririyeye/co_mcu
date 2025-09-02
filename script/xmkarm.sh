
#!/bin/bash
BASEDIR=$(dirname "$0")
cd $BASEDIR/..

export XMAKE_GLOBALDIR=$(pwd)

xmake g --network=private

xmake f -p cross \
    --toolchain=m4-arm-none-eabi \
    -y -vD \
    --USING_NET=n --USING_EXAMPLE=n \
    -m minsizerel \
    -P .
xmake -vD
xmake install -o install

