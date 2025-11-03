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
#include "Commands/TREX.h"

#include "../Helpers/Console/CommandLineParser.h"
#include "../Shell/Shell.h"
#include "Commands/NetworkIO.h"
#include "Commands/NetworkTools.h"
#include "Commands/QueryBenchmark.h"

using namespace Shell;

int main(int argc, char **argv) {
  std::cout << std::endl;
  std::cout << "░        ░░       ░░░        ░░  ░░░░  ░\n";
  std::cout << "▒▒▒▒  ▒▒▒▒▒  ▒▒▒▒  ▒▒  ▒▒▒▒▒▒▒▒▒  ▒▒  ▒▒\n";
  std::cout << "▓▓▓▓  ▓▓▓▓▓       ▓▓▓      ▓▓▓▓▓▓    ▓▓▓\n";
  std::cout << "████  █████  ███  ███  █████████  ██  ██\n";
  std::cout << "████  █████  ████  ██        ██  ████  █\n";
  std::cout << std::endl;

  CommandLineParser clp(argc, argv);
  pinThreadToCoreId(clp.value<int>("core", 1));
  checkAsserts();
  ::Shell::Shell shell;

  new ApplyPartitionFile(shell);
  new RAPTORToTREX(shell);
  new CreateCompactLayoutGraph(shell);
  new Customization(shell);
  new ShowInfoOfTREX(shell);
  new WriteTREXToCSV(shell);
  new EventDistributionOverTime(shell);
  new CheckBorderStops(shell);
  new ExportTREXTimeExpandedGraph(shell);
  new BuildTBTEGraph(shell);
  new ShowInducedCellOfNetwork(shell);

  new RunTREXQuery(shell);
  new RunTREXProfileQueries(shell);

  new RunTransitiveRAPTORQueries(shell);
  new RunOneTransitiveRAPTORQuery(shell);
  new RunTransitiveTripBasedQueries(shell);
  new RunTransitiveCSAQueries(shell);
  new RunTransitiveProfileTripBasedQueries(shell);

  new RunGeoRankedRAPTORQueries(shell);
  new RunGeoRankedTripBasedQueries(shell);
  new RunGeoRankedTREXQueries(shell);

  new IntermediateToTD(shell);
  new IntermediateToTE(shell);

  new ExportTEGraphToHubLabelFile(shell);

  new RunTDDijkstraQueries(shell);
  new RunTEDijkstraQueries(shell);

  new TEToPTL(shell);
  new RunPTLQueries(shell);

  new TEToPPTL(shell);
  new RunPPTLQueries(shell);

  new DistanceNetwork(shell);
  new StopsImportance(shell);

  shell.run();
  return 0;
}
