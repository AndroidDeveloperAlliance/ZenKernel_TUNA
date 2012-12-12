#!/tmp/busybox sh
cd /tmp
mkdir ramdisk
cd ramdisk
/tmp/busybox gzip -dc ../boot.img-ramdisk.gz | /tmp/busybox cpio -i
# If init.rc contains 'init.d' it probably already runs init scripts from there
if [ -z `/tmp/busybox grep init.d init.rc` ]; then
	echo '' >> init.rc
	echo 'service run_parts /system/sbin/bb/busybox run-parts /system/etc/init.d' >> init.rc
	echo '    class main' >> init.rc
	echo '    oneshot' >> init.rc
fi
# Cleanup existing init.rc of things we don't want it to set
/tmp/busybox sed -i '/write \/sys\/devices\/system\/cpu\/cpu0\/cpufreq\/scaling_max_freq/d' init.rc
/tmp/busybox sed -i '/write \/sys\/devices\/system\/cpu\/cpu0\/cpufreq\/scaling_min_freq/d' init.rc
/tmp/busybox sed -i '/write \/sys\/devices\/system\/cpu\/cpu0\/cpufreq\/screen_off_max_freq/d' init.rc
/tmp/busybox sed -i '/write \/sys\/devices\/system\/cpu\/cpu0\/cpufreq\/scaling_governor/d' init.rc
/tmp/busybox sed -i '/write \/sys\/devices\/system\/cpu\/cpu1\/cpufreq\/scaling_max_freq/d' init.rc
/tmp/busybox sed -i '/write \/sys\/devices\/system\/cpu\/cpu1\/cpufreq\/scaling_min_freq/d' init.rc
/tmp/busybox sed -i '/write \/sys\/devices\/system\/cpu\/cpu1\/cpufreq\/screen_off_max_freq/d' init.rc
/tmp/busybox sed -i '/write \/sys\/devices\/system\/cpu\/cpu1\/cpufreq\/scaling_governor/d' init.rc
# Insert fstab with additional mount options
cp ../fstab.tuna fstab.tuna
rm ../boot.img-ramdisk.gz
/tmp/busybox sh -c "/tmp/busybox find . | /tmp/busybox cpio -o -H newc | /tmp/busybox gzip > ../boot.img-ramdisk.gz"
cd ..

