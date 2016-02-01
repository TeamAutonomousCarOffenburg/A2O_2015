#include <iostream>

#include "RightOfWayAnalyzer.h"
#include "RightOfWayAnalyzerTests.h"

int main(int argc, char **argv) {
  
    std::cout << "Testing..." << std::endl;
    
    IRightOfWayAnalyzer* rowAnalyzer = new RightOfWayAnalyzer();
    RightOfWayAnalyzerTests* rowAnalyzerTests = new RightOfWayAnalyzerTests();
    
    int err = rowAnalyzerTests->runTests(rowAnalyzer);
    
    std::cout << "Errors: " << err << std::endl;

    return 0;
}
