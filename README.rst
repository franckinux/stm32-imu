Sources
-------

Look at .gitmodules for repositories.

Building the application
------------------------

.. code-block:: console

    git clone --recurse-submodules
    cd imu
    mkdir build
    cd build
    cmake .. -N Ninja
    ninja

Code debugging
--------------

In one console :

.. code-block:: console

    openocd -f openocd.cfg

In another console :

.. code-block:: console

    gdb-multiarch -q -x openocd.gdb build/imu-firmware.elf

