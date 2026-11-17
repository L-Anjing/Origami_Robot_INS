#include "robot_model.h"
#include "preprocess.h"
#include <tinyxml2.h>
namespace origami
{
V3D xml_vector(tinyxml2::XMLElement* e, const char* key, const char* fallback = nullptr)
{
    if (!e)
        throw std::invalid_argument("Missing XML element");
    const char* value = e->Attribute(key);
    if (!value)
        value = fallback;
    if (!value)
        throw std::invalid_argument("Missing XML vector");
    std::istringstream s(value);
    V3D v;
    s >> v.x() >> v.y() >> v.z();
    end(s);
    if (!v.allFinite())
        throw std::invalid_argument("Nonfinite geometry");
    return v;
}
Model load_robot_model(const std::string& path, const Json& c)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("Cannot parse model");
    auto root = doc.FirstChildElement("mujoco");
    if (!root)
        throw std::invalid_argument("Expected MJCF");
    auto option = root->FirstChildElement("option");
    if (!option || std::abs(option->DoubleAttribute("timestep") - .001) > 1e-12 ||
        (xml_vector(option, "gravity") - V3D(0, 0, -9.81)).norm() > 1e-10)
        throw std::invalid_argument("Unsupported model time/gravity");
    auto world = root->FirstChildElement("worldbody");
    auto named = [](tinyxml2::XMLElement* parent, const char* tag, const std::string& name)
    {
        if (!parent)
            throw std::invalid_argument("Missing XML parent");
        for (auto e = parent->FirstChildElement(tag); e; e = e->NextSiblingElement(tag))
            if (e->Attribute("name") && name == e->Attribute("name"))
                return e;
        throw std::invalid_argument("Missing model element: " + name);
    };
    auto body = named(world, "body", "plate0");
    Model model;
    model.json["xml_initial_root"] = array(xml_vector(body, "pos"));
    Json vertices = Json::array();
    for (int n = 0; n < 6; ++n)
    {
        for (auto key : {"quat", "euler", "axisangle", "xyaxes", "zaxis"})
            if (body->Attribute(key))
                throw std::invalid_argument("Nonidentity model rest rotation");
        auto geom = named(body, "geom", "geom" + std::to_string(n));
        if (!geom->Attribute("type") || std::string(geom->Attribute("type")) != "box")
            throw std::invalid_argument("Expected box geometry");
        V3D center = xml_vector(geom, "pos", "0 0 0"), size = xml_vector(geom, "size");
        V3D dimensions(c.at("length_m"), c.at("thickness_m"), c.at("width_m"));
        if ((2 * size - dimensions).norm() > 1e-10)
            throw std::invalid_argument("Physical dimensions mismatch");
        Json plate = Json::array();
        int signs[8][3] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                           {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
        for (auto& sign : signs)
            plate.push_back(array(V3D(center + size.cwiseProduct(V3D(sign[0], sign[1], sign[2])))));
        vertices.push_back(plate);
        if (n < 5)
        {
            auto child = named(body, "body", "plate" + std::to_string(n + 1));
            model.offsets[n] = xml_vector(child, "pos");
            auto joint = child->FirstChildElement("joint");
            if (!joint || !joint->Attribute("type") ||
                std::string(joint->Attribute("type")) != "hinge" ||
                xml_vector(joint, "pos", "0 0 0").norm() > 1e-10)
                throw std::invalid_argument("Unsupported joint");
            body = child;
        }
    }
    model.offsets[5] = vector3(c.at("logged_terminal_offset_S_m"));
    model.json["plate_vertices"] = vertices;
    model.json["marker_link_offsets"] = Json::array();
    for (auto& v : model.offsets)
        model.json["marker_link_offsets"].push_back(array(v));
    model.json["cracks"] = Json::array();
    for (auto name : {"crack1", "crack2"})
    {
        auto e = named(world, "geom", name);
        model.json["cracks"].push_back({{"name", name},
                                        {"center", array(xml_vector(e, "pos"))},
                                        {"half_size", array(xml_vector(e, "size"))}});
    }
    return model;
}
} // namespace origami
