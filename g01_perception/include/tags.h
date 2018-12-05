//
// Created by rig8f on 29/11/18.
//

#ifndef G01_PERCEPTION_TAGS_H
#define G01_PERCEPTION_TAGS_H

#include <math.h>

const std::vector<std::string> tagnames = {
        "red_cube_1", "red_cube_2", "red_cube_3", "red_cube_4",
        "yellow_cyl_1", "yellow_cyl_2",
        "green_triangle_1", "green_triangle_2", "green_triangle_3",
        "blue_cube_1", "blue_cube_2", "blue_cube_3", "blue_cube_4",
        "red_triangle_1", "red_triangle_2", "red_triangle_3"
};
const std::vector<std::vector<std::string>> linknames = {
        {"cube1",          "cube1_link"},
        {"cube2",          "cube2_link"},
        {"cube3",          "cube3_link"},
        {"cube4",          "cube4_link"},
        {"Hexagon0",       "Hexagon0_link"},
        {"Hexagon1",       "Hexagon1_link"},
        {"Triangle0",      "Triangle0_link"},
        {"Triangle1",      "Triangle1_link"},
        {"Triangle2",      "Triangle2_link"},
        {"blue_cube_1",    "blue_cube_1_link"},
        {"blue_cube_2",    "blue_cube_2_link"},
        {"blue_cube_3",    "blue_cube_3_link"},
        {"blue_cube_4",    "blue_cube_4_link"},
        {"red_triangle_1", "red_triangle_1_link"},
        {"red_triangle_2", "red_triangle_2_link"},
        {"red_triangle_3", "red_triangle_3_link"}};

// todo tune here
const float CUBE_LEN = 0.098; // length of cube
const float CYL_HEIGHT = 2 * CUBE_LEN; // height of cylinder, twice the cube
const float TRI_SECTION = CUBE_LEN * sqrt(2); // largest base of triangle: L*sqrt(2) (hyp of half cube)
const float TRI_HEIGHT = TRI_SECTION / 2; // height of tri, half of largest base

inline std::vector<float> getVolume(int tag_id) {
    std::vector<float> vol;
    if (tag_id < 0)
        return vol; // error case
    else if (tag_id < 4) { // cube
        vol.emplace_back(CUBE_LEN);
        vol.emplace_back(CUBE_LEN);
        vol.emplace_back(CUBE_LEN);
    } else if (tag_id < 6) { // cyl
        vol.emplace_back(CUBE_LEN);
        vol.emplace_back(CUBE_LEN);
        vol.emplace_back(CYL_HEIGHT);
    } else if (tag_id < 9) { // tri
        vol.emplace_back(TRI_SECTION);
        vol.emplace_back(TRI_SECTION);
        vol.emplace_back(TRI_HEIGHT);
    } else if (tag_id < 13) { // cube
        vol.emplace_back(CUBE_LEN);
        vol.emplace_back(CUBE_LEN);
        vol.emplace_back(CUBE_LEN);
    } else if (tag_id < 16) { // tri
        vol.emplace_back(TRI_SECTION);
        vol.emplace_back(TRI_SECTION);
        vol.emplace_back(TRI_HEIGHT);
    }
    return vol;
}

inline std::vector<float> getVolume(std::string tag_name) {
    int index = -1;
    auto it = std::find(tagnames.begin(), tagnames.end(), tag_name);
    if (it != tagnames.end())
        index = std::distance(tagnames.begin(), it);
    return getVolume(index);
}

#endif //G01_PERCEPTION_TAGS_H
