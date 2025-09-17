# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(SRCS main.c 
						mpu_9250_i2c.c 
						algo/madgwick.c 
						algo/quaternion.c 
						algo/rotation_vector.c
						algo/param.c
						algo/euclidean.c
                    INCLUDE_DIRS
                    ".")