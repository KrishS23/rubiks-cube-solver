#include <bits/stdc++.h>
#include "Solver/DFSSolver.h"
#include "Solver/BFSSolver.h"
#include "Solver/IDDFSSolver.h"
#include "Solver/IDAstarSolver.h"
#include "PatternDatabases/CornerDBMaker.h"

using namespace std;

int main() {
    string fileName = "C:\\Users\\user\\CLionProjects\\rubiks-cube-solver\\Databases\\cornerDepth5V1.txt";

    RubiksCubeBitboard cube;
    auto shuffleMoves = cube.randomShuffleCube(13);
    cube.print();
    for (auto move: shuffleMoves) cout << cube.getMove(move) << " ";
    cout << "\n";

    IDAstarSolver<RubiksCubeBitboard, HashBitboard> idaStarSolver(cube, fileName);
    auto moves = idaStarSolver.solve();

    idaStarSolver.rubiksCube.print();
    for (auto move: moves) cout << cube.getMove(move) << " ";
    cout << "\n";

    return 0;
}
