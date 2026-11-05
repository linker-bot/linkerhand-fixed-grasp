#ifndef LINKERHAND_GRASP_TYPES_H
#define LINKERHAND_GRASP_TYPES_H
struct Grap_Obj {
    int ID;
    int count;
    float x, y, z, rx, ry, rz;
};
struct ObjectData {
    int id = 0;
    float cen_x = 0, cen_y = 0, cen_z = 0;
    float vec_x = 0, vec_y = 0, vec_z = 0;
    float a = 0, b = 0, c = 0, d = 0;
    int count = 0;
};
#endif
