# Introduction #

This is a tutorial on building an x86 (32- or 64-bit) Linux kernel with the Xenomai patches on Ubuntu Linux. The end result is a debian package which, when installed, properly registers itself with the Ubuntu package management system so that you don't clutter up your machine.

The following is adapted from:
  * https://help.ubuntu.com/community/Kernel/Compile#Alternate_Build_Method:_The_Old-Fashioned_Debian_Way
  * http://www.xenomai.org/documentation/xenomai-2.6/README.INSTALL

# Details #

### Download and install Ubuntu ###

Ubuntu 10.04 _Lucid Lynx_ is the current Ubuntu LTS (Long-Term Support) release, and an install iso can be found here: http://releases.ubuntu.com/lucid/

### Package Pre-reqs ###

```
sudo apt-get install kernel-package
sudo apt-get install fakeroot build-essential crash kexec-tools makedumpfile kernel-wedge
sudo apt-get build-dep linux
sudo apt-get install git-core libncurses5 libncurses5-dev libelf-dev asciidoc binutils-dev
sudo apt-get install qt3-dev-tools libqt3-mt-dev 
```

### Set up convenience variables ###

First navigate to an empty directory to act as the workspace. Then set up these environment variables and directories:
```
export linux_version=2.6.35.9
export linux_tree=`pwd`/linux-$linux_version
```
```
export xenomai_version=2.6.0
export xenomai_root=`pwd`/xenomai-$xenomai_version
```
```
export build_root=`pwd`/build
mkdir $build_root
```

### Download and unpack Xenomai ###

Xenomai 2.6.0 is the current latest stable release. It supports the following kernels on x86 architectures and has been tested by us with the following Ubuntu distributions:

| Xenomai Version | Mainline Kernel | Ubuntu 10.04 LTS |
|:----------------|:----------------|:-----------------|
| 2.6.0 | 2.6.35.9 | √ |
| 2.5.6 | 2.6.35.9 | √ |

For more information on the kernels used by various Ubuntu distributions, see: http://kernel.ubuntu.com/~kernel-ppa/info/kernel-version-map.html

```
wget http://download.gna.org/xenomai/stable/xenomai-$xenomai_version.tar.bz2
tar xf xenomai-$xenomai_version.tar.bz2
```

### Download and unpack Linux Kernel Source ###

( For more info on building kernels in debian, see https://help.ubuntu.com/community/Kernel/Compile#AltBuildMethod )

First, download the linux kernel sources, and untar them:
```
wget http://www.kernel.org/pub/linux/kernel/v2.6/linux-$linux_version.tar.bz2
tar xf linux-$linux_version.tar.bz2
```
Second, copy the kernel config that your machine is already using as the basis for the real-time kernel:
```
cp -vi /boot/config-`uname -r` $linux_tree/.config
```

### Patch the Kernel ###
```
$xenomai_root/scripts/prepare-kernel.sh --arch=x86\
  --adeos=$xenomai_root/ksrc/arch/x86/patches/adeos-ipipe-2.6.35.9-x86-2.8-04.patch\
  --linux=$linux_tree
```

### Configure the Kernel ###
```
cd $linux_tree
make menuconfig
```
The following are a few options to check:
  * `Real-time sub-system`
    * ENABLE `Xenomai`
  * `Power management and ACPI options`
    * `ACPI (Advanced Configuration and Power Interface) Support`
      * DISABLE `Processor`
    * `CPU Frequency scaling`
      * DISABLE `CPU Frequency scaling`
  * `Processor Type and Features`
    * SET `Processor family`

**NOTE:** If your kernel does not boot, and gets stuck at "Xenomai: debug mode enabled" try being more agressive with disabling kernel options suggested [on the Xenomai wiki](http://www.xenomai.org/index.php/FAQs#Which_kernel_settings_should_be_avoided.3F)

**NOTE:** If you plan on using RTSocketCAN or other hardware interfaces, you need to enable these manually under `Real-time sub-system`.

<a href='Hidden comment: 
make bzImage modules
'></a>


Now you can build the kernel into a debian package and install it:
```
export CONCURRENCY_LEVEL=7
fakeroot make-kpkg --bzimage --initrd --append-to-version=-xenomai-$xenomai_version kernel-image kernel-headers modules
cd ..
sudo dpkg -i linux-image-*.deb
sudo dpkg -i linux-headers-*.deb
```
The package doesn't create an initrd, so you will then need to run:
```
sudo update-initramfs -c -k "$linux_version-xenomai-$xenomai_version"
sudo update-grub
```

### Configure Xenomai User Libraries ###

If you actually want to write code that uses xenomai, you will need to build and install the xenomai userspace libs.

```
cd $build_root
$xenomai_root/configure\
  --enable-shared\
  --enable-smp\
  --enable-posix-auto-mlockall\
  --enable-dlopen-skins\
  --enable-x86-sep
make
sudo make install
```

### Add xenomai Group For Non-root RT ###

First add the "xenomai" group.
```
sudo addgroup xenomai
```

Then, add the users who will be allowed to use realtime tasks to the group. For the current user you can simply execute:
```
sudo adduser `whoami` xenomai
```

Then, add the xenomai GID to the xenomai configuration. First, get the group id:
```
cat /etc/group | sed -nr "s/xenomai:.:([0-9]+):.*/\1/p"
```
Then, modify the grub defaults (located at /etc/default/grub) so that"GRUB\_CMD\_LINE\_LINUX\_DEFAULT" has at least the following arguments:
```
GRUB_CMDLINE_LINUX_DEFAULT="xeno_nucleus.xenomai_gid=<YOUR_GID>"
```
Then update grub:
```
sudo update-grub
```


### Other Bits ###

#### IO-APIC ####
Depending on your hardware, you might need to reconfigure grub after trying to boot. If you get the error:
```
MP-BIOS bug: 8254 timer not connected to IO-APIC
```
Then you need to add the "noapic" option to your grub config defaults (located at /etc/default/grub). To do this in Grub2, add "noapic" to "GRUB\_CMD\_LINE\_LINUX\_DEFAULT" so that it looks like this:
```
GRUB_CMDLINE_LINUX_DEFAULT="splash noapic"
```
Then update grub:
```
sudo update-grub
```

#### Nouveau ####
If your system gets through the boot sequence (shown as long as you don't have "quiet" enabled in the grub config) and then the screen goes black, it might be a problem with the Nouveau graphics driver. In this case, modify the grub config defaults (located at /etc/default/grub) so that "GRUB\_CMD\_LINE\_LINUX\_DEFAULT" reads like so:

```
GRUB_CMDLINE_LINUX_DEFAULT="splash noapic rdblacklist=nouveau nouveau.modeset=0"
```
Then update grub:
```
sudo update-grub
```