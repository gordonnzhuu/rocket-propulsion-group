= MINOS =

https://phab.burpg.space/project/board/190/

== What is MINOS for, and why are we building it? ==
- MINOS is meant to be a peice of firmware for BURPG operations that can run on MoTE4, as well as any future boards we build, specifically MoTE5 and Daedalus
- MINOS stands for **M**INOS **Is** the **N**ew **O**perating **S**ystem
- The goal is to create a multithreaded system using FreeRTOS that is reliable and configurable enough to be reused for any potential DAQ, Control, and Flight Software needs

== What was wrong with Motesoft Firmware? ==
- It was a mess! It was all in one giant file, it had a lot of stuff hardcoded, much of the configuration system was poorly designed, and it was single-threaded
- Its config system was inconsistent and limited
- Any kind of logic/control system was hardcoded

== Design Philosophy ==
- Multithreaded, but in a way that is deterministic and not at risk of random crashes.
- Compatible with multiple boards, specifically Teensy4.1 and STM32F4
    - Should consist of "abstract layer", which handles parallelism, logic, networking, etc. and "driver layer", which handles interacting with hardware components
    - Abstract Layer must work universally, Driver Layer can be board-specific. Control which driver layer code is use with C Macros
- Multipurpose, Maintainable, and Configurable. The design for MoTE4 firmware was "screw it, just make it work for our one use case ASAP". We have learned our lesson (?)

== TODO List for MinOS ==
- Design a new packed encoding scheme. Goals:
    - Capable of configuring more than just sensors and actuators. Minimum of SENSOR, ACTUATOR, and CONFIG datatypes.
    - Capable of returning more sophisticated sensor data than just a 32 bit INT.
        - JSON + Base64 encoded packets maybe? Pros: highly configurable, human readable, null terminated, so can be of variable length. Can have one big UDP packet rather than lots of little ones. Cons: Higher computational overhead, not compatible with MoTE encoding
    - Wishlist: Backwards compatible with MoTE4 packet encoding?
- Impelement thread-safe network capability via UDP, similar to the one currently on MoTE4
- Design a general purpose DAQ protocol that can interface with MoTE4 sensors **and** the sensors that will be on MoTE5/Daedalus (see schematic)
    - Should be thread safe, and optimized to parallelize around hardware delays (e.g. waiting for ADC to multiplex)
    - Should be designed abstractly, so we can write/change the calls to the actual ADC later as hardware specs change
- More stuff to come, this will probably be replaced by a phab task board in future


