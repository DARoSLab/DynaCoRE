### RobotSystems file converter ###

def main():
# TELLO_DynaCtrl_Definition.h
    with open("TELLO_DynaCtrl_Definition.h", "rt") as fin:
        with open("TELLO_DynaCtrl_Definition_a.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_DynaCtrl_Definition_a.h", "rt") as fin:
        with open("TELLO_DynaCtrl_Definition_b.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_DynaCtrl_Definition_b.h", "rt") as fin:
        with open("TELLO_DynaCtrl_Definition_c.h", "wt") as fout:
            for line in fin:
                fout.write(line.replace('humanoid','tello'))

# TELLO_StateEstimator.cpp
    with open("TELLO_StateEstimator.cpp", "rt") as fin:
        with open("TELLO_StateEstimator_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_StateEstimator_a.cpp", "rt") as fin:
        with open("TELLO_StateEstimator_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_StateEstimator_b.cpp", "rt") as fin:
        with open("TELLO_StateEstimator_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('humanoid','tello'))

# TELLO_StateEstimator.hpp
    with open("TELLO_StateEstimator.hpp", "rt") as fin:
        with open("TELLO_StateEstimator_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_StateEstimator_a.hpp", "rt") as fin:
        with open("TELLO_StateEstimator_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_StateEstimator_b.hpp", "rt") as fin:
        with open("TELLO_StateEstimator_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('humanoid','tello'))

# TELLO_StateProvider.cpp
    with open("TELLO_StateProvider.cpp", "rt") as fin:
        with open("TELLO_StateProvider_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_StateProvider_a.cpp", "rt") as fin:
        with open("TELLO_StateProvider_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_StateProvider_b.cpp", "rt") as fin:
        with open("TELLO_StateProvider_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('humanoid','tello'))

# TELLO_StateProvider.hpp
    with open("TELLO_StateProvider.hpp", "rt") as fin:
        with open("TELLO_StateProvider_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_StateProvider_a.hpp", "rt") as fin:
        with open("TELLO_StateProvider_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_StateProvider_b.hpp", "rt") as fin:
        with open("TELLO_StateProvider_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('humanoid','tello'))

# TELLO_interface.cpp
    with open("TELLO_interface.cpp", "rt") as fin:
        with open("TELLO_interface_a.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_interface_a.cpp", "rt") as fin:
        with open("TELLO_interface_b.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_interface_b.cpp", "rt") as fin:
        with open("TELLO_interface_c.cpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('humanoid','tello'))

# TELLO_interface.hpp
    with open("TELLO_interface.hpp", "rt") as fin:
        with open("TELLO_interface_a.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_interface_a.hpp", "rt") as fin:
        with open("TELLO_interface_b.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("TELLO_interface_b.hpp", "rt") as fin:
        with open("TELLO_interface_c.hpp", "wt") as fout:
            for line in fin:
                fout.write(line.replace('humanoid','tello'))

# CMakeLists.txt
    with open("CMakeLists.txt", "rt") as fin:
        with open("CMakeLists_a.txt", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("CMakeLists_a.txt", "rt") as fin:
        with open("CMakeLists_b.txt", "wt") as fout:
            for line in fin:
                fout.write(line.replace('Humanoid','TELLO'))
    with open("CMakeLists_b.txt", "rt") as fin:
        with open("CMakeLists_c.txt", "wt") as fout:
            for line in fin:
                fout.write(line.replace('humanoid','tello'))
 
### TEMPLATE
## tello_v3_no_head.urdf
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('Humanoid','TELLO'))
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('Humanoid','TELLO'))
    # with open("", "rt") as fin:
        # with open("", "wt") as fout:
            # for line in fin:
                # fout.write(line.replace('humanoid','tello'))
main()
