
#!/bin/bash
BASEDIR=$(dirname "$0")
cd $BASEDIR/..

export XMAKE_GLOBALDIR=$(pwd)

xmake g --network=private

xmake_opts=()

# 将命令行中形如 --gd32=y / --gd32_test=y 直接透传
for arg in "$@"; do
    case "$arg" in
        --gd32=*|--gd32_test=*) xmake_opts+=("$arg");;
    esac
done

# 支持通过环境变量 GD32 / GD32_TEST=1 启用
if [ "${GD32}" = "1" ] && ! printf '%s\n' "${xmake_opts[@]}" | grep -q '^--gd32='; then
    xmake_opts+=("--gd32=y")
fi
if [ "${GD32_TEST}" = "1" ] && ! printf '%s\n' "${xmake_opts[@]}" | grep -q '^--gd32_test='; then
    xmake_opts+=("--gd32_test=y")
fi

xmake f -p cross \
    --toolchain=m4-arm-none-eabi \
    -y -vD \
    --USING_NET=n --USING_EXAMPLE=n \
    -m minsizerel \
    -P . "${xmake_opts[@]}"
xmake -vD
xmake install -o install

