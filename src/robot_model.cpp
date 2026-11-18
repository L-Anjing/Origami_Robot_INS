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
Quaterniond body_rotation(tinyxml2::XMLElement* body, bool degrees)
{
    if (const char* text = body->Attribute("quat"))
    {
        std::istringstream in(text);
        double w, x, y, z;
        in >> w >> x >> y >> z;
        end(in);
        Quaterniond q(w, x, y, z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-12)
            throw std::invalid_argument("Invalid rest quaternion");
        return q.normalized();
    }
    if (body->Attribute("euler"))
    {
        V3D e = xml_vector(body, "euler");
        if (degrees)
            e *= pi / 180;
        return Quaterniond(Eigen::AngleAxisd(e.x(), V3D::UnitX()) *
                           Eigen::AngleAxisd(e.y(), V3D::UnitY()) *
                           Eigen::AngleAxisd(e.z(), V3D::UnitZ()));
    }
    if (const char* text = body->Attribute("axisangle"))
    {
        std::istringstream in(text);
        V3D a;
        double angle;
        in >> a.x() >> a.y() >> a.z() >> angle;
        end(in);
        if (!a.allFinite() || a.norm() < 1e-12 || !std::isfinite(angle))
            throw std::invalid_argument("Invalid axis-angle");
        return Quaterniond(Eigen::AngleAxisd(degrees ? angle * pi / 180 : angle, a.normalized()));
    }
    if (body->Attribute("xyaxes") || body->Attribute("zaxis"))
        throw std::invalid_argument("Use quat/euler/axisangle rest rotation");
    return Quaterniond::Identity();
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
    auto compiler = root->FirstChildElement("compiler");
    bool degrees = !compiler || !compiler->Attribute("angle") ||
                   std::string(compiler->Attribute("angle")) == "degree";
    if (compiler && compiler->Attribute("eulerseq") &&
        std::string(compiler->Attribute("eulerseq")) != "xyz")
        throw std::invalid_argument("Unsupported Euler sequence; use quaternions");
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
    model.physical_length_m = c.at("length_m");
    model.json["xml_initial_root"] = array(xml_vector(body, "pos"));
    Json vertices = Json::array();
    for (int n = 0; n < 6; ++n)
    {
        body_rotation(body, degrees);
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
                std::string(joint->Attribute("type")) != "hinge")
                throw std::invalid_argument("Unsupported joint");
            auto& h = model.joints[n];
            if (!joint->Attribute("name"))
                throw std::invalid_argument("Unnamed joint");
            h.name = joint->Attribute("name");
            h.parent = n;
            h.child = n + 1;
            h.translation = model.offsets[n];
            h.anchor_child = xml_vector(joint, "pos", "0 0 0");
            h.rest_rotation = body_rotation(child, degrees);
            h.axis_child = xml_vector(joint, "axis", "0 0 1");
            if (h.axis_child.norm() < 1e-12)
                throw std::invalid_argument("Zero joint axis");
            h.axis_child.normalize();
            h.axis_parent = h.rest_rotation * h.axis_child;
            h.reference = joint->DoubleAttribute("ref", 0) * (degrees ? pi / 180 : 1);
            h.limited = joint->BoolAttribute("limited", joint->Attribute("range") != nullptr);
            if (const char* text = joint->Attribute("range"))
            {
                std::istringstream range(text);
                range >> h.lower >> h.upper;
                end(range);
                if (degrees)
                {
                    h.lower *= pi / 180;
                    h.upper *= pi / 180;
                }
                if (!std::isfinite(h.lower) || !std::isfinite(h.upper) || h.lower > h.upper)
                    throw std::invalid_argument("Invalid joint range");
            }
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
