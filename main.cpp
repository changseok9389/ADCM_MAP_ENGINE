#include "map_parser.h"

int main(){

    map_parser map = map_parser("/tmp/tmp.216LQpgm9W/semantic_map.pb");
    map.set_resolution_multiplier(3);
    map.load_map();
    map.parse_map();
    map.draw_map();
    map.show_map_debug(224);
    return 0;
}
