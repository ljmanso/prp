INCLUDE_DIRECTORIES(/home/harit/workspace/gsoc16/caffe-master/include /usr/local/cuda-7.5/targets/x86_64-linux/include )
SET( LIBS ${LIBS} -L/home/harit/workspace/gsoc16/caffe-master/install/lib -L/usr/local/cuda-7.5/targets/x86_64-linux/lib
-L/usr/lib/x86_64-linux-gnu -lcaffe  -lcublas -lboost_system -lglog -lgflags -lm -lboost_filesystem -lprotobuf -lcudart)
