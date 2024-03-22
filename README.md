# This is an absolute mess at the moment. 

This library is adapted from [this](https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1) GitHub repository. I tried to bring over everything that is needed, but it still is asking for RC inputs. Currently, it moves the servos in a promising way without maxing them out, but that's it.

Note some header files are included in the 'include' folder, including the one that sets servo pins.

The upside to this way of doing things is it would be easy to add new gaits and change between different gaits while operating. No MATLAB is needed as all the math is done on the arduino, resulting in smoother and more diverse movement. If we can get this version working, it would likely be the best. We may also be able to have RC capability for more easily testing multidirectional movement without testing sensors.

This code is also based on a hexapod with a more circular base, where each side of the hexagon is equal. Our hexapod sort of has a front, while his does not.

```
    Ours                    His
   _____                   ___
  |  -> | front           /   \
  |_____|                 \___/
```