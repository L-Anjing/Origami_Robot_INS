#include "mat_export.h"
#include "common_lib.h"
#include <cstdint>
#include <cstring>
#include <fstream>
namespace origami
{
namespace
{
using Bytes = std::vector<char>;
template <class T> void append(Bytes& b, T value)
{
    auto p = reinterpret_cast<const char*>(&value);
    b.insert(b.end(), p, p + sizeof(T));
}
void element(Bytes& b, uint32_t type, const Bytes& data)
{
    append(b, type);
    append(b, uint32_t(data.size()));
    b.insert(b.end(), data.begin(), data.end());
    while (b.size() % 8)
        b.push_back(0);
}
Bytes chars(const std::string& s)
{
    return Bytes(s.begin(), s.end());
}
Json numeric(std::vector<int> dims)
{
    size_t n = 1;
    for (int d : dims)
        n *= d;
    return {{"_dims", dims}, {"_data", std::vector<double>(n, 0.)}};
}
Json column(const Json& values)
{
    auto out = numeric({int(values.size()), 1});
    out["_data"] = values;
    return out;
}
void put(Json& a, size_t offset, const Json& values)
{
    for (size_t i = 0; i < values.size(); ++i)
        a["_data"][offset + i] = values.at(i);
}
Bytes matrix(const std::string& name, const Json& value)
{
    Bytes body, flags, dims;
    uint32_t cls = value.is_object()   ? (value.contains("_dims") ? 6 : 2)
                   : value.is_string() ? 4
                   : value.is_array()  ? 1
                                       : 6;
    append(flags, cls);
    append(flags, uint32_t(0));
    element(body, 6, flags);
    std::vector<int> shape = {1, 1};
    if (value.is_object() && value.contains("_dims"))
        shape = value.at("_dims").get<std::vector<int>>();
    else if (value.is_string())
        shape = {1, int(value.get<std::string>().size())};
    else if (value.is_array())
        shape = {1, int(value.size())};
    for (int d : shape)
        append(dims, int32_t(d));
    element(body, 5, dims);
    element(body, 1, chars(name));
    if (cls == 2)
    {
        Bytes length;
        append(length, int32_t(64));
        element(body, 5, length);
        Bytes names;
        for (auto it = value.begin(); it != value.end(); ++it)
        {
            if (it.key().size() >= 64)
                throw std::runtime_error("MAT field name too long");
            auto field = chars(it.key());
            field.resize(64, 0);
            names.insert(names.end(), field.begin(), field.end());
        }
        element(body, 1, names);
        for (auto it = value.begin(); it != value.end(); ++it)
        {
            auto nested = matrix("", it.value());
            body.insert(body.end(), nested.begin(), nested.end());
        }
    }
    else if (cls == 1)
    {
        for (const auto& v : value)
        {
            auto nested = matrix("", v);
            body.insert(body.end(), nested.begin(), nested.end());
        }
    }
    else if (cls == 4)
    {
        Bytes data;
        for (unsigned char ch : value.get<std::string>())
            append(data, uint16_t(ch));
        element(body, 4, data);
    }
    else
    {
        Bytes data;
        if (value.is_object())
            for (auto& v : value.at("_data"))
                append(data, v.get<double>());
        else
            append(data, value.is_boolean() ? (value.get<bool>() ? 1. : 0.) : value.get<double>());
        element(body, 9, data);
    }
    Bytes result;
    element(result, 14, body);
    return result;
}
} // namespace
void export_mat(const std::string& folder)
{
    uint16_t endian = 1;
    if (*reinterpret_cast<char*>(&endian) != 1)
        throw std::runtime_error("MAT writer requires little endian host");
    std::ifstream summaryFile(folder + "/summary.json");
    Json summary;
    summaryFile >> summary;
    if (!summary.contains("model"))
        throw std::invalid_argument("MAT schema 2 export requires simulation output");
    int n = summary.at("samples");
    const auto& c = summary.at("config");
    const auto& model = summary.at("model");
    Json payload = {{"schema_version", 2},
                    {"time_s", numeric({1, n})},
                    {"controls", numeric({3, n})},
                    {"gravity_W_m_s2", column(Json{0, 0, -c.at("gravity_m_s2").get<double>()})},
                    {"truth_endpoints_W", numeric({3, 7, n})},
                    {"default_reference_index", c.at("reference_imu_index")},
                    {"policy_names", Json{"none", "glrt", "phase"}},
                    {"covariance_state_order", "dp,dv,dtheta_local,dbg,dba"},
                    {"imu_body_offset_S_m", numeric({3, 6})},
                    {"initial_root_W_m", column(c.at("robot").at("initial_root_W_m"))},
                    {"physical_tip_offset_S_m",
                     column(Json{c.at("robot").at("geometry").at("length_m"), 0, 0})},
                    {"plate_vertices_S_m", numeric({3, 8, 6})},
                    {"crack_centers_W_m", numeric({3, 2})},
                    {"crack_half_sizes_m", numeric({3, 2})}};
    auto policies = c.at("robot").at("zupt_modes");
    int defaultPolicy = 0;
    for (int k = 0; k < 3; ++k)
        if (policies[k] == c.at("robot").at("default_zupt_mode"))
            defaultPolicy = k;
    payload["default_policy_one_based"] = defaultPolicy + 1;
    Json indices = Json::array();
    for (auto it = summary.at("references").begin(); it != summary.at("references").end(); ++it)
        indices.push_back(std::stoi(it.key().substr(1)));
    payload["reference_indices_zero_based"] = numeric({1, int(indices.size())});
    put(payload["reference_indices_zero_based"], 0, indices);
    payload["truth_endpoint_index_zero_based"] = numeric({1, 6});
    for (int k = 0; k < 6; ++k)
        payload["truth_endpoint_index_zero_based"]["_data"][k] =
            c.at("truth").at("endpoint_indices")[k].is_null()
                ? -1
                : c.at("truth").at("endpoint_indices")[k].get<int>();
    for (int k = 0; k < 6; ++k)
        for (int j = 0; j < 8; ++j)
            put(payload["plate_vertices_S_m"], 3 * j + 24 * k, model.at("plate_vertices")[k][j]);
    for (int k = 0; k < 2; ++k)
    {
        put(payload["crack_centers_W_m"], 3 * k, model.at("cracks")[k].at("center"));
        put(payload["crack_half_sizes_m"], 3 * k, model.at("cracks")[k].at("half_size"));
    }
    payload["plate_faces_one_based"] = numeric({6, 4});
    int faces[6][4] = {{1, 2, 3, 4}, {5, 6, 7, 8}, {1, 2, 6, 5},
                       {2, 3, 7, 6}, {3, 4, 8, 7}, {4, 1, 5, 8}};
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 4; ++j)
            payload["plate_faces_one_based"]["_data"][i + 6 * j] = faces[i][j];
    Json mode = {{"q_WS_wxyz", numeric({4, 6, n})},
                 {"R_WS", numeric({3, 3, 6, n})},
                 {"relative_markers_W", numeric({3, 7, n})},
                 {"orientation_source", summary.at("mode") == "ground_truth"
                                            ? "simulation attitude, debug only"
                                            : "six-axis AHRS; no truth correction"},
                 {"references", Json::object()}};
    for (auto& index : indices)
    {
        std::string key = "S" + std::to_string(index.get<int>());
        Json ref;
        for (auto field : {"raw_position_W", "raw_velocity_W"})
            ref[field] = numeric({3, n});
        ref["raw_q_WS_wxyz"] = numeric({4, n});
        for (auto field :
             {"eskf_position_W", "eskf_velocity_W", "gyro_bias_rad_s", "accel_bias_m_s2"})
            ref[field] = numeric({3, n, 3});
        ref["eskf_q_WS_wxyz"] = numeric({4, n, 3});
        ref["covariance_diagonal"] = numeric({15, n, 3});
        for (auto field : {"zupt_updated", "zupt_requested", "nis_rejected", "nis"})
            ref[field] = numeric({3, n});
        for (auto field :
             {"glrt_candidate", "phase_glrt_candidate", "glrt_statistic", "glrt_valid"})
            ref[field] = numeric({1, n});
        ref["quality"]["valid_for_aiding"] = numeric({1, n});
        mode["references"][key] = ref;
    }
    std::ifstream frames(folder + "/frames.jsonl");
    std::string line;
    int k = 0;
    while (std::getline(frames, line))
    {
        if (k >= n)
            throw std::runtime_error("Too many frames");
        Json frame = Json::parse(line);
        payload["time_s"]["_data"][k] = frame.at("time_s");
        put(payload["controls"], 3 * k, frame.at("controls"));
        for (int j = 0; j < 7; ++j)
        {
            put(payload["truth_endpoints_W"], 3 * j + 21 * k, frame.at("truth_endpoints")[j]);
            put(mode["relative_markers_W"], 3 * j + 21 * k, frame.at("shape")[j]);
        }
        for (int j = 0; j < 6; ++j)
        {
            auto q = frame.at("quaternion")[j];
            put(mode["q_WS_wxyz"], 4 * j + 24 * k, q);
            M3D R = Quaterniond(q[0].get<double>(), q[1].get<double>(), q[2].get<double>(),
                                q[3].get<double>())
                        .toRotationMatrix();
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    mode["R_WS"]["_data"][a + 3 * b + 9 * j + 54 * k] = R(a, b);
        }
        for (auto& index : indices)
        {
            auto key = "S" + std::to_string(index.get<int>());
            auto& ref = mode["references"][key];
            auto& input = frame.at("references").at(key);
            put(ref["raw_position_W"], 3 * k, input.at("raw").at("position"));
            put(ref["raw_velocity_W"], 3 * k, input.at("raw").at("velocity"));
            put(ref["raw_q_WS_wxyz"], 4 * k, input.at("raw").at("quaternion"));
            bool quiet = input.at("glrt_candidate");
            ref["glrt_candidate"]["_data"][k] = int(quiet);
            ref["phase_glrt_candidate"]["_data"][k] =
                int(quiet && frame.at("phase_candidate").get<bool>());
            ref["glrt_valid"]["_data"][k] = input.value("glrt_valid", false) ? 1 : 0;
            ref["glrt_statistic"]["_data"][k] = input.value("glrt_statistic", 0.);
            ref["quality"]["valid_for_aiding"]["_data"][k] = input.value("valid", true) ? 1 : 0;
            for (int policy = 0; policy < 3; ++policy)
            {
                auto& state = input.at("policies")[policy];
                for (auto mapping : {std::pair{"position", "eskf_position_W"},
                                     {"velocity", "eskf_velocity_W"},
                                     {"bg", "gyro_bias_rad_s"},
                                     {"ba", "accel_bias_m_s2"},
                                     {"quaternion", "eskf_q_WS_wxyz"},
                                     {"covariance_diagonal", "covariance_diagonal"}})
                {
                    auto values = state.at(mapping.first);
                    int d = values.size();
                    put(ref[mapping.second], d * k + d * n * policy, values);
                }
                ref["zupt_updated"]["_data"][policy + 3 * k] =
                    state.at("accepted").get<bool>() ? 1 : 0;
                ref["zupt_requested"]["_data"][policy + 3 * k] =
                    state.value("requested", false) ? 1 : 0;
                ref["nis_rejected"]["_data"][policy + 3 * k] =
                    state.at("rejected_nis").get<bool>() ? 1 : 0;
                ref["nis"]["_data"][policy + 3 * k] = state.at("nis");
            }
        }
        ++k;
    }
    if (k != n)
        throw std::runtime_error("Incomplete frames");
    payload["modes"][summary.at("mode").get<std::string>()] = mode;
    std::ofstream out(folder + "/robot_odometry.mat", std::ios::binary);
    Bytes header(128, 0);
    std::string title = "MATLAB 5.0 MAT-file, Origami INS native C++";
    std::copy(title.begin(), title.end(), header.begin());
    header[124] = 0;
    header[125] = 1;
    header[126] = 'I';
    header[127] = 'M';
    out.write(header.data(), header.size());
    for (auto it = payload.begin(); it != payload.end(); ++it)
    {
        auto bytes = matrix(it.key(), it.value());
        out.write(bytes.data(), bytes.size());
    }
    if (!out)
        throw std::runtime_error("MAT write failed");
}
} // namespace origami
