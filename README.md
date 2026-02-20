[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
# T-REX
> Transfer - Ranked EXploration

This repo contains the code for T-REX, a journey planning algorithm I developed during my master thesis.
The underlying code base is taken from [FLASH-TB](https://github.com/TransitRouting/FLASH-TB), which is based on [ULTRA](https://github.com/kit-algo/ULTRA).

If you use this repository, please cite my work using

```
@mastersthesis{Steil2025_1000190843,
    author       = {Steil, Patrick},
    year         = {2025},
    title        = {{A} {M}ultilevel {D}esign for {S}calable {P}areto-{O}ptimal {P}ublic {T}ransit {Q}ueries},
    doi          = {10.5445/IR/1000190843},
    publisher    = {{Karlsruher Institut für Technologie (KIT)}},
    pagetotal    = {90},
    school       = {Karlsruher Institut für Technologie (KIT)},
    language     = {english}
}
```

This code contains the following journey planning algorithms:

 - RAPTOR (Round-bAsed Public Transit Optimized Router)
 - CSA (Connection Scan Algorithm)
 - TB (Trip-Based Public Transit Routing)
 - T-REX (Transfer-Ranked Exploration)
 - TP (Transfer Patterns)
 - TDD (Time-Dependent Dijkstra)
 - TED (Time-Expanded Dijkstra)
 - PTL (Public Transit Labeling)

as well as some ULTRA-Extensions for RAPTOR, TB and CSA. For more information, check out [ULTRA](https://github.com/kit-algo/ULTRA).

## How to use

Clone the repo and all submodules using

    git clone --recurse-submodules https://github.com/PatrickSteil/TREX
as we use some submodules in `ExternalLibs`.

The executables are located in `Runnables` and are built using Makefile.

    make all

or e.g., 

    make TREXRelease
 which builds a binary with all optimizations turned on.
 
The executables provide a command line interface in which commands are executed. `help` lists all possible commands, and `help [command]` explains how to use them.

## Example
Inside `Runnables` is a helper bash script `downloadExample.sh` to download and extract a GTFS file (which models the Karlsruher Verkehrsverbund KVV). 
After building all executables and running `downloadExample.sh`, the following steps create a raptor and trip based binary:

Inside `./Network`:

```
parseGTFS ../Datasets/Karlsruhe/GTFS/ ../Datasets/Karlsruhe/gtfs.binary
gtfsToIntermediate ../Datasets/Karlsruhe/gtfs.binary 20250101 20250102 false false ../Datasets/Karlsruhe/intermediate.binary
makeOneHopTransfers ../Datasets/Karlsruhe/intermediate.binary 86400 ../Datasets/Karlsruhe/intermediate.binary true
intermediateToRAPTOR ../Datasets/Karlsruhe/intermediate.binary ../Datasets/Karlsruhe/raptor.binary
```
To transform raptor into T-REX (with e.g., 4 Levels) , use these commands inside `./TREX`:

```
raptorToTREX ../Datasets/Karlsruhe/raptor.binary ../Datasets/Karlsruhe/trex.binary 4
createCompactLayoutGraph ../Datasets/Karlsruhe/trex.binary ../Datasets/Karlsruhe/compact_layout.graph
```
The last command created the compact layout graph, which you can partition using e.g, MT-KaHyPar:
```
>>> ./../ExternalLibs/mt-kahypar/build/mt-kahypar/application/MtKaHyPar \
    -h [COMPACT_LAYOUT_GRAPH] \
    -k [1<< NUM_LEVELS e.g., 16] \
    -e [IMBALANCE e.g., 0.2] \
    -o cut \
    -m rb \
    -t 6 \
    --write-partition-file true \
    --partition-output-folder [OUTPUTFOLDER] \
    --input-file-format metis \
    --instance-type=graph \
    --preset-type highest_quality \
    --i-r-flow-algo flow_cutter \
    --i-r-refine-until-no-improvement true
```

Then, again inside `./TREX`:

```
applyPartitionFile ../Datasets/Karlsruhe/compact_layout.graph.metis.part16.epsilon0.2.seed0.KaHyPar 4 ../Datasets/Karlsruhe/trex.binary
customize ../Datasets/Karlsruhe/trex.binary ../Datasets/Karlsruhe/trex.binary
showInfoOfTREX ../Datasets/Karlsruhe/trex.binary
runTREXQueries ../Datasets/Karlsruhe/trex.binary 1000
runTREXProfileQueries ../Datasets/Karlsruhe/trex.binary 1000
runTransitiveTripBasedQueries ../Datasets/Karlsruhe/trex.binary.trip 1000
runTransitiveRAPTORQueries /home/patrick/Documents/MLTB/Datasets/Karlsruhe/trex.binary.trip.raptor 1000
```

which for example yields on my machine the following output:
```
> applyPartitionFile ../Datasets/Karlsruhe/compact_layout.graph.metis.part16.epsilon0.2.seed0.KaHyPar 4 ../Datasets/Karlsruhe/trex.binary
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.raptor.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
(...)
Read 4,345 many IDs!
> customize ../Datasets/Karlsruhe/trex.binary ../Datasets/Karlsruhe/trex.binary
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.raptor.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
(...)
Starting Level 0 [IBEs: 46585]...
100.00% (5s 328ms)
done!
Starting Level 1 [IBEs: 22821]...
100.00% (3s 369ms)  4ms)1s 344ms)
done!
Starting Level 2 [IBEs: 13614]...
100.00% (2s 935ms)
done!
Starting Level 3 [IBEs: 1956]...
100.00% (554ms)
done!
******* Stats *******
Number of collected IBEs: 46,585.00
Collect IBEs: 466µs
Sort IBEs: 257ms 481µs
Filter IBEs: 891µs
Total time: 12s 447ms 860µs
> showInfoOfTREX ../Datasets/Karlsruhe/trex.binary
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.raptor.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
(...)
** Number of Local Transfers **
Level 0: 6,447,122  73.55 %
Level 1: 1,179,182  13.45 %
Level 2: 535,401  6.11 %
Level 3: 546,389  6.23 %
Level 4: 57,959  0.66 %
> runTREXQueries ../Datasets/Karlsruhe/trex.binary 1000
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.raptor.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
(...)
Rounds: 6.71
Scanned trips: 3,824.38
Scanned stops: 29,179.41
Relaxed transfers: 102,709.38
Enqueued trips: 102,757.41
Added journeys: 5.53
Number of discarded edges: 2,874.63
Scan initial transfers: 0µs
Evaluate initial transfers: 9µs
Scan trips: 880µs
Total time: 894µs
Avg. Journeys: 1.60
> runTREXProfileQueries ../Datasets/Karlsruhe/trex.binary 1000
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.raptor.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
(...)
Rounds: 1,117.16
Scanned trips: 45,611.87
Scanned stops: 243,884.16
Relaxed transfers: 1,338,110.83
Enqueued trips: 1,338,776.62
Added journeys: 85.82
Scan initial transfers: 0µs
Get Journeys: 501µs
Scan trips: 10ms 573µs
Total time: 11ms 306µs
Avg. journeys: 40.67
> runTransitiveTripBasedQueries ../Datasets/Karlsruhe/trex.binary.trip 1000
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.raptor.graph
Loading static graph from ../Datasets/Karlsruhe/trex.binary.trip.graph
(...)
Rounds: 7.28
Scanned trips: 7,528.06
Scanned stops: 48,461.74
Relaxed transfers: 178,015.40
Enqueued trips: 178,063.43
Added journeys: 5.53
Distance / MaxSpeed: 0.00
Scan initial transfers: 0µs
Evaluate initial transfers: 9µs
Scan trips: 1ms 410µs
Total time: 1ms 425µs
Avg. journeys: 1.60
> runTransitiveRAPTORQueries /home/patrick/Documents/MLTB/Datasets/Karlsruhe/trex.binary.trip.raptor 1000
Loading static graph from /home/patrick/Documents/MLTB/Datasets/Karlsruhe/trex.binary.trip.raptor.graph
(...)
Statistics:
Round Routes Segments  Edges Stops (trip) Stops (transfer) Init  Collect Scan  Transfers  Total
clear  0  0  0  0  0  0µs  0µs  0µs  0µs  1µs
init  0  0  1  1  1  6µs  0µs  0µs  0µs  7µs
0 47  716  141  115 97  8µs  0µs  8µs  2µs 20µs
1  611 10,046  850  964  433  7µs 32µs  105µs 12µs  158µs
2  2,101 34,889  1,474  2,068  647  6µs  149µs  337µs 24µs  518µs
3  2,937 48,633  914  1,525  405  7µs  238µs  439µs 19µs  706µs
4  2,346 36,821  326  573  151  6µs  166µs  324µs  9µs  507µs
5  1,249 18,270 89  163 43  7µs 67µs  160µs  3µs  239µs
6  501  7,042 19 38 10  6µs 21µs 61µs  1µs 89µs
7  160  2,174  3  7  2  4µs  5µs 18µs  0µs 28µs
8 43  590  0  1  0  1µs  1µs  4µs  0µs  7µs
9  8  107  0  0  0  0µs  0µs  0µs  0µs  1µs
10  1 30  0  0  0  0µs  0µs  0µs  0µs  0µs
11  0 15  0  0  0  0µs  0µs  0µs  0µs  0µs
12  0  0  0  0  0  0µs  0µs  0µs  0µs  0µs
total 10,004  159,333  3,817  5,455  1,789 63µs  684µs  1ms 461µs 73µs  2ms 287µs
Total time: 2ms 288µs
Avg. rounds: 7.29
Avg. journeys: 1.60
```
