#!/usr/bin/env rosh
import yaml
zone_controls = {}
wait_for_service('list_locations')
for loc in services.list_locations().locations:
    for dev in services.list_devices(loc.locationId).devices:
        zone_controls['ZONE%s' % dev.deviceId] = []
        for cont in services.get_device_info(dev.deviceId).controls:
            zone_controls['ZONE%s' % dev.deviceId].append({
            'location'  : loc.name,
            'device'    : dev.name,
            'deviceId'  : dev.deviceId,
            'controlId' : cont.controlId,
            'control'   : cont.name,
            # 'zone'      : ''
            })

print yaml.dump(zone_controls)