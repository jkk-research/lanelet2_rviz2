#include "lanelet2_rviz2/rapidxml.hpp"
#include "lanelet2_rviz2/rapidxml_utils.hpp"
#include "lanelet2_rviz2/node.hpp"
#include "lanelet2_rviz2/way.hpp"
#include "lanelet2_rviz2/relation.hpp"
#include "lanelet2_rviz2/map.hpp"
#include <iostream>
#include <map>

int main() {
    // File to parse
    const char* filename = "/home/zahu/ros2_ws/src/lanelet2_rviz2/src/ZalaZone_Uni_track_full_early.osm";
    rapidxml::file<> xmlFile(filename);
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    // Containers for nodes and ways
    std::map<int, Node*> nodes;
    std::map<int, Way*> ways;

    // Root node
    rapidxml::xml_node<>* root = doc.first_node("osm");

    // Parse MetaInfo
    Map map;
    map.osm_generator = root->first_attribute("generator")->value();
    rapidxml::xml_node<>* metaInfoNode = root->first_node("MetaInfo");
    if (metaInfoNode) {
        map.metainfo_format_version = std::stoi(metaInfoNode->first_attribute("format_version")->value());
        map.metainfo_map_version = metaInfoNode->first_attribute("map_version")->value();
    }

    // Parse nodes
    for (rapidxml::xml_node<>* node = root->first_node("node"); node; node = node->next_sibling("node")) {
        int id = std::stoi(node->first_attribute("id")->value());
        double lat = std::stod(node->first_attribute("lat")->value());
        double lon = std::stod(node->first_attribute("lon")->value());
        double local_x = 0, local_y = 0, ele = 0;

        // Parse tags
        for (rapidxml::xml_node<>* tag = node->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
            std::string key = tag->first_attribute("k")->value();
            double value = std::stod(tag->first_attribute("v")->value());
            if (key == "local_x") local_x = value;
            else if (key == "local_y") local_y = value;
            else if (key == "ele") ele = value;
        }

        Node* newNode = new Node(id, lat, lon, local_x, local_y, ele);
        nodes[id] = newNode;
    }

    // Parse ways
    for (rapidxml::xml_node<>* wayNode = root->first_node("way"); wayNode; wayNode = wayNode->next_sibling("way")) {
        int id = std::stoi(wayNode->first_attribute("id")->value());
        Way* way = new Way(id);

        // Parse nd references
        for (rapidxml::xml_node<>* nd = wayNode->first_node("nd"); nd; nd = nd->next_sibling("nd")) {
            int ref = std::stoi(nd->first_attribute("ref")->value());
            if (nodes.count(ref)) {
                way->add_node(nodes[ref]);
            }
        }

        ways[id] = way;
    }

    // Parse relations
    for (rapidxml::xml_node<>* relationNode = root->first_node("relation"); relationNode; relationNode = relationNode->next_sibling("relation")) {
        int id = std::stoi(relationNode->first_attribute("id")->value());
        Way* left = nullptr;
        Way* right = nullptr;
        std::string type, subtype, location;
        int speed_limit = 0;
        bool one_way = false;

        // Parse members
        for (rapidxml::xml_node<>* member = relationNode->first_node("member"); member; member = member->next_sibling("member")) {
            std::string role = member->first_attribute("role")->value();
            int ref = std::stoi(member->first_attribute("ref")->value());
            if (role == "left" && ways.count(ref)) {
                left = ways[ref];
            } else if (role == "right" && ways.count(ref)) {
                right = ways[ref];
            }
        }

        // Parse tags
        for (rapidxml::xml_node<>* tag = relationNode->first_node("tag"); tag; tag = tag->next_sibling("tag")) {
            std::string key = tag->first_attribute("k")->value();
            std::string value = tag->first_attribute("v")->value();
            if (key == "type") type = value;
            else if (key == "subtype") subtype = value;
            else if (key == "speed_limit") speed_limit = std::stoi(value);
            else if (key == "location") location = value;
            else if (key == "one_way") one_way = (value == "yes");
        }

        Relation* relation = new Relation(id, left, right, type, subtype, speed_limit, location, one_way);
        map.relations.push_back(relation);
    }

    // std::cout << "Parsed map with " << nodes.size() << " nodes, " << ways.size() << " ways, and " 
    //           << map.relations.size() << " relations.\n";

    // // print data from first relation
    // if (!map.relations.empty()) {
    //     Relation* relation = map.relations.front();
    //     std::cout << "Relation " << relation->id() << ":\n";
    //     std::cout << "  Left: " << relation->left()->id() << "\n";
    //     std::cout << "  Right: " << relation->right()->id() << "\n";
    //     std::cout << "  Type: " << relation->type() << "\n";
    //     std::cout << "  Subtype: " << relation->subtype() << "\n";
    //     std::cout << "  Speed Limit: " << relation->speed_limit() << "\n";
    //     std::cout << "  Location: " << relation->location() << "\n";
    //     std::cout << "  One Way: " << (relation->one_way() ? "yes" : "no") << "\n";
    // }

    // Cleanup dynamically allocated memory
    for (auto& pair : nodes) delete pair.second;
    for (auto& pair : ways) delete pair.second;
    for (auto* relation : map.relations) delete relation;

    return 0;
}
