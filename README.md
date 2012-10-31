The layout of our virtual usb environment is:

Android Emulator(QEMU) connects to Host PC(QEMU).

To achieve this:
the Android Emulator's QEMU was modified for adding the virtual usb device.
--->device-qemu/android/external/qemu

The Host PC's QEMU was modified for adding the virtual usb device wrapper.
--->host-qemu

Latest work is here.

http://www.youtube.com/watch?v=F984AJL6p2Q&feature=relmfu

Now this Android emulator only works on Linux.
Machine environment we are using is as follows.

OS: Ubuntu (11.04)
CPU: Intel (R) Core (TM) 2 Duo CPU E7500@2.93GHz
Memory: 4GB

The virtual usb device specification is:

Number of endpoint  :5(EP0, 1,2,3,4)
Control transfer    :○
Interrupt transfer  :×
Bulk transfer       :○
Isochronous transfer:×

Background is:

According to the official website of Android, the Android Emulator does not support the USB connection with host PC.
For this reason, in the Android USB gadgets development, the debugging using the emulator can not be performed.
Our goal is to achieve the virtual USB connection between the host PC and the  Android Emulator!
First Step
   http://www.youtube.com/watch?v=Kd0SSdqG94U

Second Step
  http://www.youtube.com/watch?v=7LTbQ7s7XKA

Third Step
   http://www.youtube.com/watch?v=_ODy_Mm8KWw