N=$1
I=$2
D=$3
echo "Partition $D into $N with imbalance of $I"

./../ExternalLibs/mt-kahypar/build/mt-kahypar/application/MtKaHyPar \
    -h ~/Documents/Datasets/$D/TREX/compact_layout.graph.metis \
    -k $N \
    -e $I \
    -o cut \
    -m rb \
    -t 6 \
    --write-partition-file true \
    --partition-output-folder /home/patrick/Documents/Datasets/$D/TREX/kahypar \
    --input-file-format metis \
    --instance-type=graph \
    --preset-type highest_quality \
    --i-r-flow-algo flow_cutter \
    --i-r-refine-until-no-improvement true # \
    # --enable-progress-bar=true \
    # --show-detailed-timings=true \
    # --verbose=true
