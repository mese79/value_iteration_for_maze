#include <iostream>
#include <string>
#include<iomanip>

using namespace std;


void print_2d_array(auto &data2d) {
    ostringstream ssRow;
    string output;
    int minPad = 2;
    ssRow << data2d[0][0];
    int baseLen = ssRow.str().length() < 8 ? 9 : ssRow.str().length() + 1;
    ssRow.str(string());  // clear row buffer

    for (auto &row: data2d) {
        for (auto &entity: row) {
            ssRow << "|" << setw(baseLen + minPad / 2) << entity << string(minPad / 2, ' ');
        }
        ssRow << "|\n";
        output += ssRow.str() + string(ssRow.str().length() - 1, '-') + "\n";
        ssRow.str(string());  // clear row buffer
    }
    output = string(output.find("\n"), '-') + "\n" + output;
    cout << endl << output;
}
