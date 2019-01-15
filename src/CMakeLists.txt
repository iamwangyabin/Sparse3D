add_library( Sparse3D SHARED
        sift.cpp
#        ShiTomasi.cpp
        Png2Cloud.cpp
        )

target_link_libraries( Sparse3D
        ${THIRD_PARTY_LIBS}
        )
#add_executable( sift sift.cpp )