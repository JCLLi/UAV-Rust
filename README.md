
# Quadrupel Project Template

Most of the code we provide you for the Embedded Systems Lab
is not actually contained in this template. Instead, it's provided 
as a library called [`tudelft_quadrupel`](https://docs.rs/tudelft_quadrupel)

In your `Cargo.toml`, its version is set to version `1`. This means, that running 
`cargo update` will automatically get you bugfixes if released by the course staff 
(any version `1.x.x`).

You can find the assignment text on the [course website](https://cese.ewi.tudelft.nl)

## Template layout

Your template consists of two main folders: `dronecode` and `runner`. The compilation target
of these two is different. The `dronecode` code is compiled for `thumbv6m-none-eabi` while the default
target for `runner` is `x86_64-unknown-linux-gnu`.

The `dronecode` will run on your drone, and contains an example on how to write some basic code for the 
drone. `runner` is responsible for uploading the program to your drone, and can then also start any
code that needs to run on the PC to communicate with the drone.


## Fixes
- Fix for os error 13: sudo usermod -a -G dialout $USER
Restart to take effect