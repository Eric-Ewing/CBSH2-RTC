/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, March 2021
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "CBS.h"
#include <fstream>
#include <boost/filesystem.hpp>
// #include "lib/lazy-cbs-mirror/include/lazycbs/mapf-solver.h"  // Adjust the path according to actual header file location
// #include "lib/bcp-mapf/bcp/Includes.h"     // Adjust the path according to actual header file location

using std::ifstream;

void saveDependenciesFile(vector<vector<double>> dependencies){
	ofstream f("dependencies.txt");
	for (int i = 0; i < dependencies.size(); i++){
		for (int j = 0; j < dependencies[i].size(); j++){
			f << i << "," << j << "," << dependencies[i][j] << endl;
		}
	}
}

void saveComponentsFile(vector<vector<int>> components, string filenameComponents, double time, string filenameTime){
	// if (boost::filesystem::exists(filenameComponents)){
	// 	cout << filenameComponents << endl;
	// 	return;
	// }
	// else{
		// cout << filenameComponents << endl;
		ofstream f(filenameComponents.c_str());
		for (auto component : components){
			for (int agent : component){
				f << agent << ", ";
			}
			f << endl;
		}
	// }
	ofstream g(filenameTime.c_str());
	g << "Decomp time: " << time << std::endl;
}

vector<vector<int>> loadComponentsFile(const string &filename) {
    vector<vector<int>> components;
    ifstream f(filename.c_str());

    if (!f) {
        cerr << "Error opening file for reading: " << filename << endl;
        exit(1); // Return an empty vector in case of an error
    }

    int agent;
    while (f >> agent) {
        char comma;
        vector<int> component;

        do {
            component.push_back(agent);
        } while (f >> comma >> agent && comma == ',');
        components.push_back(component);
    }

    return components;
}

double loadTimingFile(const string &filename) {
	ifstream f(filename.c_str());
	double decompTime = 0;
	if (!f) {
        cerr << "Error opening file for reading: " << filename << endl;
        exit(1); // Return an empty vector in case of an error
    }

    f >> decompTime;
	return decompTime;
}

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("output,o", po::value<string>(), "output file for schedule")
		("outputPaths", po::value<string>(), "output file for paths")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(60), "cutoff time (seconds)")
		("nodeLimit", po::value<int>()->default_value(MAX_NODES), "node limit")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("stats", po::value<bool>()->default_value(false), "write to files some statistics")
		("agentIdx", po::value<string>()->default_value(""), "customize the indices of the agents (e.g., \"0,1\")")

		("componentsOnly,c", po::value<bool>()->default_value(false), "only compute decomposition...")

		// params for instance generators
		("rows", po::value<int>()->default_value(0), "number of rows")
		("cols", po::value<int>()->default_value(0), "number of columns")
		("obs", po::value<int>()->default_value(0), "number of obstacles")
		("warehouseWidth", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instances")

		// params for CBS
		("heuristics", po::value<string>()->default_value("WDG"), "heuristics for the high-level search (Zero, CG,DG, WDG)")
		("prioritizingConflicts", po::value<bool>()->default_value(true), "conflict priortization. If true, conflictSelection is used as a tie-breaking rule.")
		("bypass", po::value<bool>()->default_value(true), "Bypass1")
		("disjointSplitting", po::value<bool>()->default_value(false), "disjoint splitting")
		("rectangleReasoning", po::value<string>()->default_value("GR"), "rectangle reasoning strategy (None, R, RM, GR, Disjoint)")
		("corridorReasoning", po::value<string>()->default_value("GC"), " corridor reasoning strategy (None, C, PC, STC, GC, Disjoint")
		("mutexReasoning", po::value<bool>()->default_value(false), "Using mutex reasoning")
		("targetReasoning", po::value<bool>()->default_value(true), "Using target reasoning")
		("restart", po::value<int>()->default_value(1), "number of restart times (at least 1)")
		("sipp", po::value<bool>()->default_value(false), "using sipp as the single agent solver")
		("decompose", po::value<bool>()->default_value(false), "perform instance decomposition")
		("threshold", po::value<double>()->default_value(0), "Threshold for dependency")
		("ID", po::value<bool>()->default_value(false), "Use Independence Detection Algorithm")
		;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help"))
	{
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);
	/////////////////////////////////////////////////////////////////////////
	/// check the correctness and consistence of params
    //////////////////////////////////////////////////////////////////////
	heuristics_type h;
	if (vm["heuristics"].as<string>() == "Zero")
		h = heuristics_type::ZERO;
	else if (vm["heuristics"].as<string>() == "CG")
		h = heuristics_type::CG;
	else if (vm["heuristics"].as<string>() == "DG")
		h = heuristics_type::DG;
	else if (vm["heuristics"].as<string>() == "WDG")
		h = heuristics_type::WDG;
	else
	{
		cout << "WRONG heuristics strategy!" << endl;
		return -1;
	}

	rectangle_strategy r;
	if (vm["rectangleReasoning"].as<string>() == "None")
		r = rectangle_strategy::NR;  // no rectangle reasoning
	else if (vm["rectangleReasoning"].as<string>() == "R")
		r = rectangle_strategy::R;  // rectangle reasoning for entire paths
	else if (vm["rectangleReasoning"].as<string>() == "RM")
		r = rectangle_strategy::RM;  // rectangle reasoning for path segments
    else if (vm["rectangleReasoning"].as<string>() == "GR")
        r = rectangle_strategy::GR;  // generalized rectangle reasoning
	else if (vm["rectangleReasoning"].as<string>() == "Disjoint")
		r = rectangle_strategy::DR; // disjoint rectangle reasoning
	else
	{
		cout << "WRONG rectangle reasoning strategy!" << endl;
		return -1;
	}

	corridor_strategy c;
	if (vm["corridorReasoning"].as<string>() == "None")
		c = corridor_strategy::NC;  // no corridor reasoning
	else if (vm["corridorReasoning"].as<string>() == "C")
		c = corridor_strategy::C;  // corridor reasoning
    else if (vm["corridorReasoning"].as<string>() == "PC")
        c = corridor_strategy::PC;  // corridor + pseudo-corridor reasoning
    else if (vm["corridorReasoning"].as<string>() == "STC")
        c = corridor_strategy::STC;  // corridor with start-target reasoning
    else if (vm["corridorReasoning"].as<string>() == "GC")
        c = corridor_strategy::GC;  // generalized corridor reasoning = corridor with start-target + pseudo-corridor
	else if (vm["corridorReasoning"].as<string>() == "Disjoint")
		c = corridor_strategy::DC; // disjoint corridor reasoning
	else
	{
		cout << "WRONG corridor reasoning strategy!" << endl;
		return -1;
	}


	///////////////////////////////////////////////////////////////////////////
	/// load the instance
    //////////////////////////////////////////////////////////////////////
	Instance originalInstance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["agentNum"].as<int>(), vm["agentIdx"].as<string>(),
		vm["rows"].as<int>(), vm["cols"].as<int>(), vm["obs"].as<int>(), vm["warehouseWidth"].as<int>());

	srand(vm["seed"].as<int>());
	int runs = vm["restart"].as<int>();
	if (vm["ID"].as<bool>()){
		cout << "Starting ID solver..." << endl;
		CBS cbs(originalInstance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		// CBS cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
		cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
		cbs.setBypass(vm["bypass"].as<bool>());
		cbs.setRectangleReasoning(r);
		cbs.setCorridorReasoning(c);
		cbs.setHeuristicType(h);
		cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
		cbs.setMutexReasoning(vm["mutexReasoning"].as<bool>());
		cbs.setSavingStats(vm["stats"].as<bool>());
		cbs.setNodeLimit(vm["nodeLimit"].as<int>());

		cbs.IDSolve(vm["cutoffTime"].as<double>());
		if(!cbs.validateSolution()){
			cout << "ID Failed..." << endl;
		}
	}
	else if (vm["componentsOnly"].as<bool>()){
		CBS cbs(originalInstance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		// CBS cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
		cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
		cbs.setBypass(vm["bypass"].as<bool>());
		cbs.setRectangleReasoning(r);
		cbs.setCorridorReasoning(c);
		cbs.setHeuristicType(h);
		cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
		cbs.setMutexReasoning(vm["mutexReasoning"].as<bool>());
		cbs.setSavingStats(vm["stats"].as<bool>());
		cbs.setNodeLimit(vm["nodeLimit"].as<int>());
		
		double dependencyThreshold = vm["threshold"].as<double>();
		
		string cacheComponentsFile = "thesis_components/" + vm["agents"].as<string>() + "_" + std::to_string(vm["agentNum"].as<int>())+"_"+std::to_string(vm["threshold"].as<double>())+ ".components";
		string timingComponentsFile = "thesis_components/" + vm["agents"].as<string>() + "_" + std::to_string(vm["agentNum"].as<int>())+"_"+std::to_string(vm["threshold"].as<double>())+ ".timing";
		ifstream f(cacheComponentsFile.c_str());
		double decompTime = 0;
		clock_t start = clock();

		vector<vector<int>> componentsOfAgents = cbs.getDependencies(vm["cutoffTime"].as<double>()*12, dependencyThreshold);
		// decompTime = (double) (clock() - start) / CLOCKS_PER_SEC;
		// componentsOfAgents = cbs.getComponents(dependencies, dependencyThreshold, vm["cutoffTime"].as<double>()*12);
		decompTime = (double) (clock() - start) / CLOCKS_PER_SEC;
		saveComponentsFile(componentsOfAgents, cacheComponentsFile, decompTime, timingComponentsFile);
		exit(0);
	}
	else if (vm["decompose"].as<bool>()){
		//////////////////////////////////////////////////////////////////////
		/// initialize the solver
		//////////////////////////////////////////////////////////////////////

		CBS cbs(originalInstance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		// CBS cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
		cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
		cbs.setBypass(vm["bypass"].as<bool>());
		cbs.setRectangleReasoning(r);
		cbs.setCorridorReasoning(c);
		cbs.setHeuristicType(h);
		cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
		cbs.setMutexReasoning(vm["mutexReasoning"].as<bool>());
		cbs.setSavingStats(vm["stats"].as<bool>());
		cbs.setNodeLimit(vm["nodeLimit"].as<int>());
		
		double dependencyThreshold = vm["threshold"].as<double>();
		
		string cacheComponentsFile = "thesis_components/" + vm["agents"].as<string>() + "_" + std::to_string(vm["agentNum"].as<int>())+"_"+std::to_string(vm["threshold"].as<double>())+ ".components";
		string timingComponentsFile = "thesis_components/" + vm["agents"].as<string>() + "_" + std::to_string(vm["agentNum"].as<int>())+"_"+std::to_string(vm["threshold"].as<double>())+ ".timing";
		ifstream f(cacheComponentsFile.c_str());
		double decompTime = 0;
		clock_t start = clock();

		vector<vector<int>> componentsOfAgents = cbs.getDependencies(vm["cutoffTime"].as<double>()*12, dependencyThreshold);
		// decompTime = (double) (clock() - start) / CLOCKS_PER_SEC;
		// componentsOfAgents = cbs.getComponents(dependencies, dependencyThreshold, vm["cutoffTime"].as<double>()*12);
		decompTime = (double) (clock() - start) / CLOCKS_PER_SEC;
		saveComponentsFile(componentsOfAgents, cacheComponentsFile, decompTime, timingComponentsFile);
		start = clock(); // just to test how we do without decomp...
		int min_f_value = 0;
		vector<CBS> solvers = vector<CBS>();

		for (vector<int> component : componentsOfAgents){
			Instance sub_instance = originalInstance.subInstance(component);

			CBS cbsSubinstance = CBS(sub_instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
			// cbsSubinstance cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
			cbsSubinstance.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
			cbsSubinstance.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
			cbsSubinstance.setBypass(vm["bypass"].as<bool>());
			cbsSubinstance.setRectangleReasoning(r);
			cbsSubinstance.setCorridorReasoning(c);
			cbsSubinstance.setHeuristicType(h);
			cbsSubinstance.setTargetReasoning(vm["targetReasoning"].as<bool>());
			cbsSubinstance.setMutexReasoning(vm["mutexReasoning"].as<bool>());
			cbsSubinstance.setSavingStats(vm["stats"].as<bool>());
			cbsSubinstance.setNodeLimit(vm["nodeLimit"].as<int>());

			cbsSubinstance.solve(vm["cutoffTime"].as<double>(), 0); // solve
			if (!cbsSubinstance.solution_found){
				cout << "Solving Failed Subinstance" << endl;
				cbsSubinstance.saveResults(vm["output"].as<string>(), vm["agents"].as<string>()+":"+ vm["agentIdx"].as<string>());
				ofstream f(timingComponentsFile.c_str());
				f << -1 << std::endl;
			}
			else{
				min_f_value += cbsSubinstance.min_f_val; //update min f value
				solvers.emplace_back(cbsSubinstance); // update all solvers
				cbs.initialRuntime += cbsSubinstance.runtime; // update runtime
				cbs.fromSubinstance(cbsSubinstance, component); // add solved paths of sub-instances
				cbs.maxCompRuntime = max(cbsSubinstance.runtime, cbs.maxCompRuntime);
				ofstream f;
				f.open(timingComponentsFile.c_str(), std::ios::app);
				f << cbsSubinstance.runtime << std::endl;
				f.close();
			}
		}
		start = clock();
		cbs.decompTime = decompTime;
		cbs.decomp = true;
		cbs.decompThreshold = dependencyThreshold;
		//////////////////////////////////////////////////////////////////////
		/// run
		//////////////////////////////////////////////////////////////////////
		double runtime = 0;
		cout << "solving entire thing" << endl;
		cbs.solve(vm["cutoffTime"].as<double>(), min_f_value);
		cbs.solution_found &= cbs.validateSolution();
		

		// bool valid = cbs2.combineSubInstances(cbs);
		//////////////////////////////////////////////////////////////////////
		/// write results to files
		//////////////////////////////////////////////////////////////////////
		if (vm.count("output"))
			cbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>()+":"+ vm["agentIdx"].as<string>());
			// cbs.saveCT(vm["output"].as<string>() + ".tree"); // for debug
		if (vm["stats"].as<bool>())
		{
			cbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>() + ":" + vm["agentIdx"].as<string>());
		}
		if (cbs.solution_found && vm.count("outputPaths"))
			cbs.savePaths(vm["outputPaths"].as<string>());
		cbs.clearSearchEngines();
		if (!cbs.solution_found){
			cout << "failed! " << vm["map"].as<string>() << ", " << vm["agents"].as<string>() << ", " << endl;
		}
		else{
			cbs.printResults();
		}
		return 0;
	}
	else{
		//////////////////////////////////////////////////////////////////////
		/// initialize the solver
		//////////////////////////////////////////////////////////////////////
		CBS cbs(originalInstance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		// CBS cbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		cbs.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
		cbs.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
		cbs.setBypass(vm["bypass"].as<bool>());
		cbs.setRectangleReasoning(r);
		cbs.setCorridorReasoning(c);
		cbs.setHeuristicType(h);
		cbs.setTargetReasoning(vm["targetReasoning"].as<bool>());
		cbs.setMutexReasoning(vm["mutexReasoning"].as<bool>());
		cbs.setSavingStats(vm["stats"].as<bool>());
		cbs.setNodeLimit(vm["nodeLimit"].as<int>());

		//////////////////////////////////////////////////////////////////////
		/// run
		//////////////////////////////////////////////////////////////////////
		double runtime = 0;
		int min_f_val = 0;
		for (int i = 0; i < runs; i++)
		{
			cbs.clear();
			cout << "Starting to solve" << endl;
			cbs.solve(vm["cutoffTime"].as<double>(), min_f_val);
			cout << "solved!" << endl;
			runtime += cbs.runtime;
			if (cbs.solution_found)
				break;
			min_f_val = (int) cbs.min_f_val; 
			cbs.randomRoot = true;
		}
		cbs.runtime = runtime;

		// bool valid = cbs.combineSubInstances(cbs);
		//////////////////////////////////////////////////////////////////////
		/// write results to files
		//////////////////////////////////////////////////////////////////////
		if (vm.count("output"))
			cbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>()+":"+ vm["agentIdx"].as<string>());
			// cbs.saveCT(vm["output"].as<string>() + ".tree"); // for debug
		if (vm["stats"].as<bool>())
		{
			cbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>() + ":" + vm["agentIdx"].as<string>());
		}
		if (cbs.solution_found && vm.count("outputPaths"))
			cbs.savePaths(vm["outputPaths"].as<string>());
		cbs.clearSearchEngines();
		cbs.printResults();
		return 0;
	}
}