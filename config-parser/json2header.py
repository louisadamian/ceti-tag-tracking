import argparse
import sys
import json


def main():
    parser = argparse.ArgumentParser(
        description='Generate binary header output')
    parser.add_argument('-i', '--input', required=True, help='Input file')
    # parser.add_argument('-o', '--out', required=True , help='Output file')
    # parser.add_argument('-v', '--var', required=True , help='Variable name to use in file')

    args = parser.parse_args()
    if not args:
        return 1

    with open(args.input, 'r') as f:
        data = json.load(f)

    ## Write the includes to start with
    out = []
    out.append('/* AN AUTOGENERATED HEADER FILE DO NOT MODIFY */')
    out.append('#ifndef _GENERATED_H_')
    out.append('#define _GENERATED_H_')
    out.append('#include <stdbool.h>')
    out.append('#include <stdint.h>')

    out.append('\nenum BoardBuildTypes { TAG = 0, FLOATER = 1, DEV = 2 };')

    out.append('\ntypedef struct startup_config_t {')
    out.append('\tbool flag;')
    out.append('\tfloat gps_wait;')
    out.append('\tuint8_t transmissions;\n} startup_config_s;')

    out.append('\ntypedef struct aprs_config_t {')
    out.append('\tchar callsign[8];')
    out.append('\tuint8_t ssid;')
    out.append('\tchar symbol[3]; //null terminated')
    out.append('\tchar dest[8];')
    out.append('\tchar digi_path[16];')
    out.append('\tuint8_t digi_ssid;')
    out.append('\tchar comment[128];')
    out.append('\tbool compressed;')
    out.append('\tuint32_t interval;')
    out.append('\tuint32_t variance;')
    out.append('\tfloat freq_lock;\n} aprs_config_s;')

    out.append('\ntypedef struct timing_config_t {')
    out.append('\tint16_t day_trigger;')
    out.append('\tint16_t night_trigger;')
    out.append('\tuint32_t day_interval;')
    out.append('\tuint32_t night_interval;')
    out.append('\tbool deep_sleep;\n} timing_config_s;')

    out.append('\ntypedef struct simulation_config_t {')
    out.append('\tfloat latlon[2];')
    out.append('\tbool sim_move;')
    out.append('\tbool aprs_tx;')
    out.append('\tbool tag_tx;\n} simulation_config_s;\n')

    for dict in data:
        if dict == "type":
            out.append(
                'static const enum BoardBuildTypes board_ = {var_name};'.
                format(var_name=data["type"].upper()))
        elif dict == "debug":
            out.append('static const bool debug_ = {var_name};'.format(
                var_name=str(data["debug"]).lower()))
        elif dict == "startup":
            out.append('static const startup_config_s startup_ = {{{flag}, {lock}, {max}}};'.format(
                flag=str(data["startup"]["flag"]).lower(), lock=data["startup"]["gps_lock_time"], max=data["startup"]["aprs_max_time"]))
        elif dict == "aprs":
            out.append(
                'static const aprs_config_s aprs_config = {{"{call_name}", {ssid}, "{symbol}", "{dest}", "{path}", {digi_ssid}, "{comment}", {format}, {time}, {var}, {freq}}};'
                .format(call_name=data["aprs"]["callsign"],
                        ssid=data["aprs"]["ssid"],
                        symbol=data["aprs"]["symbol"],
                        dest=data["aprs"]["dest"],
                        path=data["aprs"]["digi_path"],
                        digi_ssid=data["aprs"]["digi_ssid"],
                        comment=data["aprs"]["comment"],
                        format=str(data["aprs"]["comp_format"]).lower(),
                        time=(data["aprs"]["timing"] * 1000),
                        var=(data["aprs"]["variance"] * 1000),
                        freq=data["aprs"]["freq_lock"]))
        elif dict == "timing":
            out.append(
                'static const timing_config_s timing_ = {{{d_trig}, {n_trig}, {d_t}, {n_t}, {deep_sleep}}};'
                .format(d_trig=data["timing"]["day_trigger"],
                        n_trig=data["timing"]["night_trigger"],
                        d_t=(data["timing"]["day_timing"]
                             if data["timing"]["day_timing"] < 0 else
                             data["timing"]["day_timing"] * 1000),
                        n_t=(data["timing"]["night_timing"]
                             if data["timing"]["night_timing"] < 0 else
                             data["timing"]["night_timing"] * 1000),
                        deep_sleep=str(
                            data["timing"]["deep_sleep_trigger"]).lower()))
        elif dict == "simulation":
            out.append(
                'static const simulation_config_s simulation_ = {{{{{lat}, {lon}}}, {move}, {aprs_tx}, {tag_tx}}};'
                .format(lat=data["simulation"]["sim_loc"][0],
                        lon=data["simulation"]["sim_loc"][1],
                        move=str(data["simulation"]["sim_move"]).lower(),
                        aprs_tx=str(data["simulation"]["sim_aprs_tx"]).lower(),
                        tag_tx=str(data["simulation"]["sim_tag_tx"]).lower()))

    out.append('\n#endif // _GENERATED_H_')
    out = '\n'.join(out)

    with open("../generated/generated.h", 'w') as f:
        f.write(out)

    return 0


if __name__ == '__main__':
    sys.exit(main())