#include <iostream>
#include <string>
#include<iomanip>

using namespace std;


template <typename T>
void print_vector2d(vector<vector<T>> &data2d, bool flipY=true) {
    string output;
    ostringstream ssRow;
    int minPad = 2;
    // get sample entity length:
    ssRow << data2d[0][0];
    int baseLen = ssRow.str().length() < 8 ? 9 : ssRow.str().length() + 1;
    ssRow.str(string());  // clear row buffer

    // row iterator: if bottom-left is the (0, 0) then flip rows.
    auto fromIt = flipY ? data2d.end() : data2d.begin();
    auto toIt = flipY ? data2d.begin() : data2d.end();

    for (auto rowIt = fromIt; rowIt != toIt;) {
        if (flipY) {
            --rowIt;
        }

        for (T &entity: *rowIt) {
            ssRow << "|" << setw(baseLen + minPad / 2) << setprecision(5) << entity << string(minPad / 2, ' ');
        }
        ssRow << "|\n";
        output += ssRow.str() + string(ssRow.str().length() - 1, '-') + "\n";
        ssRow.str(string());  // clear row buffer

        if (!flipY) {
            ++rowIt;
        }
    }
    output = string(output.find("\n"), '-') + "\n" + output;
    cout << endl << output;
};
