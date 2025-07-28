BASEDIR=$(dirname "$0")
cd $BASEDIR/..

find . -name "*.o" -exec rm -rf {} \;
find . -name "*.cmd" -exec rm -rf {} \;
find . -name "*.ko" -exec rm -rf {} \;
find . -name "*.mod" -exec rm -rf {} \;
find . -name "*.mod.c" -exec rm -rf {} \;
find . -name "*.symvers" -exec rm -rf {} \;
find . -name "*.order" -exec rm -rf {} \;

rm .xmake -rf
rm ~/.xmake -rf
rm build -rf
rm install -rf
rm third_package/libusb/install -rf
rm third_package/libusb/libusb-1.0.26 -rf
rm app/java/com -rf
rm daemon/android/com -rf

rm ko/.xmake -rf
rm ko/build -rf
rm ko/install -rf
rm .cache -rf