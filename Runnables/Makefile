CC = g++ -fopenmp

FLAGS = -std=c++2b -pipe
OPTIMIZATION = -march=native -O3
OPTIMIZATION += -flto

DEBUG = -rdynamic -Werror -Wno-pedantic -Wall -Wextra -Wparentheses \
        -Wfatal-errors -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer

RELEASE = -ffast-math -ftree-vectorize -Wfatal-errors -DNDEBUG \
          -fomit-frame-pointer -mtune=native -fno-stack-protector \
          -mavx2 -mno-avx256-split-unaligned-load \
          -mno-avx256-split-unaligned-store -funroll-loops

PREFETCH = -DENABLE_PREFETCH -DUSE_SIMD
# SPARSEHASH = -I ../ExternalLibs/sparsehash/include

# -----------------------------
# PGO configuration
# -----------------------------
PGO_DIR = pgo-data
PGO_GEN = -fprofile-generate=$(PGO_DIR)
PGO_USE = -fprofile-use=$(PGO_DIR) -fprofile-correction

# -----------------------------
# Default targets
# -----------------------------
all: TREXRelease NetworkRelease

# -----------------------------
# Network
# -----------------------------
NetworkDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o Network Network.cpp

NetworkRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o Network Network.cpp

# -----------------------------
# TREX
# -----------------------------
TREXDebug:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) \
	      -o TREX TREX.cpp -ltbb -latomic $(PREFETCH)

TREXRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) \
	      -o TREX TREX.cpp -ltbb -latomic $(PREFETCH) -g

# -----------------------------
# TREX PGO
# -----------------------------

# Step 1: Instrumented build
TREXPGO_GEN:
	mkdir -p $(PGO_DIR)
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) $(PGO_GEN) \
	      -o TREX_pgo_gen TREX.cpp -ltbb -latomic $(PREFETCH)

# Step 2: Run workload manually
#   ./TREX_pgo_gen <args>

# Step 3: Final optimized build
TREXPGO_USE:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) $(PGO_USE) \
	      -o TREX TREX.cpp -ltbb -latomic $(PREFETCH)

# -----------------------------
# Cleanup
# -----------------------------
clean:
	rm -f TREX TREX_pgo_gen Network
	rm -rf $(PGO_DIR)

