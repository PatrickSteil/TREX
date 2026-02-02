CC=g++ -fopenmp
FLAGS=-std=c++2b -pipe
OPTIMIZATION=-march=native -O3
DEBUG=-rdynamic -Werror -Wno-pedantic -Wall -Wextra -Wparentheses -Wfatal-errors -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer
RELEASE=-ffast-math -ftree-vectorize -Wfatal-errors -DNDEBUG -fomit-frame-pointer -mtune=native -fno-stack-protector -mavx2 -mno-avx256-split-unaligned-load -mno-avx256-split-unaligned-store -funroll-loops
PREFETCH=-DENABLE_PREFETCH -DUSE_SIMD
# SPARSEHASH=-I ../ExternalLibs/sparsehash/include

all: TREXRelease NetworkRelease
# all: TPRelease TREXRelease ULTRARelease NetworkRelease
# all: TPDebug ArcTBDebug NetworkDebug ULTRADebug

NetworkDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o Network Network.cpp

NetworkRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o Network Network.cpp

ULTRADebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o ULTRA ULTRA.cpp $(PREFETCH)

ULTRARelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o ULTRA ULTRA.cpp $(PREFETCH)

ArcTBDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o ARCTB ArcTB.cpp $(PREFETCH)

ArcTBRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o ARCTB ArcTB.cpp $(PREFETCH)

TPDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) $(SPARSEHASH) -o TP TP.cpp $(PREFETCH)

TPRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) $(SPARSEHASH) -o TP TP.cpp $(PREFETCH)

TREXDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o TREX TREX.cpp -ltbb -latomic $(PREFETCH)

TREXRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o TREX TREX.cpp -ltbb -latomic $(PREFETCH)
