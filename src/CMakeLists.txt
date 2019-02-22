add_library( Sparse3D SHARED
#        sift.cpp
#        ShiTomasi.cpp
#        Png2Cloud.cpp
#        Narf.cpp
#        ColorCorrespondence.cpp
#        common_include.cpp
#        ColorGraphMatching.cpp
#        BuildCorpPointSet.cpp
#        GeometricCorrespondence.cpp
#        GeoGraphMatching.cpp
        ## opt1
#        ProgressiveOpt.cpp
#        GraphLoopDetect.cpp
        ## opt2
#        OptApp.cpp
        ## join cloudpoints
        jointPointCloud.cpp
        )

target_link_libraries( Sparse3D
        ${THIRD_PARTY_LIBS}
        )
