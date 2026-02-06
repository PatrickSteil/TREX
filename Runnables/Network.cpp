/**********************************************************************************

 Copyright (c) 2023-2025 Patrick Steil
 Copyright (c) 2019-2022 KIT ITI Algorithmics Group

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/
#include "../Helpers/Console/CommandLineParser.h"
#include "../Helpers/MultiThreading.h"
#include "../Shell/Shell.h"
#include "Commands/NetworkIO.h"
#include "Commands/NetworkTools.h"
#include "Commands/TimeTable.h"

using namespace Shell;

int main(int argc, char **argv) {
  CommandLineParser clp(argc, argv);
  pinThreadToCoreId(clp.value<int>("core", 1));
  checkAsserts();
  ::Shell::Shell shell;
  new ParseGTFS(shell);
  new GTFSToIntermediate(shell);
  new IntermediateToCSA(shell);
  new IntermediateToRAPTOR(shell);
  new IntermediateToTD(shell);
  new IntermediateToTE(shell);
  new BuildMultimodalRAPTORData(shell);
  new AddModeToMultimodalRAPTORData(shell);
  new BuildMultimodalTripBasedData(shell);
  new AddModeToMultimodalTripBasedData(shell);
  new LoadDimacsGraph(shell);
  new DuplicateTrips(shell);
  new AddGraph(shell);
  new ReplaceGraph(shell);
  new ReduceGraph(shell);
  new ReduceToMaximumConnectedComponent(shell);
  new ReduceToMaximumConnectedComponentWithTransitive(shell);
  new ApplyBoundingBox(shell);
  new ApplyCustomBoundingBox(shell);
  new MakeOneHopTransfers(shell);
  new MakeOneHopTransfersByGeoDistance(shell);
  new ApplyMaxTransferSpeed(shell);
  new ApplyConstantTransferSpeed(shell);
  new WriteIntermediateToCSV(shell);
  new WriteRAPTORToCSV(shell);
  new WriteTripBasedToCSV(shell);
  new WriteRAPTORLayoutGraphToMetis(shell);

  new RAPTORToTimeTable(shell);
  new LoadAndApplyPartition(shell);
  shell.run();
  return 0;
}
