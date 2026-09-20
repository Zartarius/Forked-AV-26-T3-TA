// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    using namespace std;

    auto log_file = ifstream(path);
    auto rows = vector<Row>();

    string timestamp_s, can_id_s, data_s, _;

    auto read_line = [&]() -> bool {
        return
            getline(log_file, _, '(') &&
            getline(log_file, timestamp_s, ')') &&
            getline(log_file, _, ')') &&
            getline(log_file, _, ' ') &&
            getline(log_file, _, ' ') &&
            getline(log_file, can_id_s, '#') &&
            getline(log_file, data_s, '\n');
    };

    auto timestamp_ff = -1.0;
    while (read_line()) {
        auto can_id = stoi(can_id_s, nullptr, 16);
        if (can_id != 512) {
            continue;
        }

        auto timestamp = stod(timestamp_s);
        timestamp_ff = (timestamp_ff >= 0.0) ? timestamp_ff : timestamp;

        for (auto i = 0; i < 8; i += 2) {
            swap(data_s[i], data_s[14 - i]);
            swap(data_s[i + 1], data_s[15 - i]);
        }
        auto data = stoull(data_s, nullptr, 16);

        auto y_measured = int16_t(data & 0xffff);
        auto u_commanded = int16_t((data >> 16) & 0xffff);

        const auto& row = (Row) {timestamp - timestamp_ff, u_commanded * 0.1, y_measured * 0.1};
        rows.push_back(row);
    }

    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
