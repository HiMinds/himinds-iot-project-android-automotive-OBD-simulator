import threading
import time
import math
import datetime
import json
import argparse
import sys
import random
from collections import namedtuple

from diagnostic_sensors import DIAGNOSTIC_SENSOR_FLOAT_CALCULATED_ENGINE_LOAD
from diagnostic_sensors import DIAGNOSTIC_SENSOR_FLOAT_ENGINE_COOLANT_TEMPERATURE
from diagnostic_sensors import DIAGNOSTIC_SENSOR_FLOAT_FUEL_PRESSURE
from diagnostic_sensors import DIAGNOSTIC_SENSOR_FLOAT_INTAKE_MANIFOLD_ABSOLUTE_PRESSURE
from diagnostic_sensors import DIAGNOSTIC_SENSOR_FLOAT_ENGINE_RPM
from diagnostic_sensors import DIAGNOSTIC_SENSOR_FLOAT_VEHICLE_SPEED
from diagnostic_sensors import DIAGNOSTIC_SENSOR_FLOAT_MAF_AIR_FLOW_RATE
from diagnostic_sensors import DIAGNOSTIC_SENSOR_FLOAT_THROTTLE_POSITION

import vhal_consts_2_0 as c

# vhal_emulator depends on a custom Python package that requires installation
# give user guidance should the import fail
try:
    from vhal_emulator import Vhal
except ImportError as e:
    isProtobuf = False
    pipTool = "pip%s" % ("3" if sys.version_info > (3, 0) else "")
    if hasattr(e, 'name'):
        if e.name == 'google': isProtobuf = True
    elif hasattr(e, 'message'):
        if e.message.endswith('symbol_database'):
            isProtobuf = True
    if isProtobuf:
        print('could not find protobuf.')
        print(
            'protobuf can be installed via "sudo %s install --upgrade protobuf"'
            % pipTool)
        sys.exit(1)
    else:
        raise e

from diagnostic_builder import DiagnosticEventBuilder


class DiagnosticHalWrapper(object):
    def __init__(self, device):
        self.vhal = Vhal(c.vhal_types_2_0, device)
        self.Interfacetoandroid = Interfacetoandroid(0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                     0, 0)

        self.Interfacetoandroid.start()

        self.liveFrameConfig = self.chat(lambda hal: hal.getConfig(
            c.VEHICLEPROPERTY_OBD2_LIVE_FRAME))
        self.freezeFrameConfig = self.chat(lambda hal: hal.getConfig(
            c.VEHICLEPROPERTY_OBD2_FREEZE_FRAME))
        self.eventTypeData = {
            'live': {
                'builder': lambda: DiagnosticEventBuilder(self.liveFrameConfig
                                                          ),
                'property': c.VEHICLEPROPERTY_OBD2_LIVE_FRAME
            },
            'freeze': {
                'builder': lambda: DiagnosticEventBuilder(self.
                                                          freezeFrameConfig),
                'property': c.VEHICLEPROPERTY_OBD2_FREEZE_FRAME
            },
        }

    def stopobdii(self):
        self.Interfacetoandroid.stop()

    def resumeobdii(self):
        self.Interfacetoandroid.start()

    def chat(self, request):
        request(self.vhal)
        return self.vhal.rxMsg()

    def put_data(self, b):
        global lastTimestamp
        self.engine = b.get_engine()
        self.coolant = b.get_coolant()
        self.fuel = b.get_fuel()
        self.intake_pressure = b.get_intake_pressure()
        self.rpm = b.get_rpm()
        self.speed = b.get_speed()
        #intake_temp = b.get_intake_temp()
        self.maf = b.get_maf()
        self.throttle_pos = b.get_throttle_pos()
        currentTimestamp = self.throttle_pos.timestamp
        assert currentTimestamp >= lastTimestamp
        if lastTimestamp != 0:
            # also, timestamps are in nanoseconds, but sleep() uses seconds
            time.sleep((currentTimestamp - lastTimestamp) / 1000000000)
        lastTimestamp = currentTimestamp
        eventType = self.engine.frame
        eventTypeData = self.eventTypeData[eventType]
        builder = eventTypeData['builder']()
        builder.setStringValue(self.engine.sensortype)
      #  builder.addIntSensor(0, 2)
      #  builder.addIntSensor(7, 18)
       # builder.addIntSensor(13, 21)
        for value in [
                self.engine, self.coolant, self.fuel, self.intake_pressure,
                self.rpm, self.speed, self.maf, self.throttle_pos
        ]:
            builder.addFloatSensor(value.id, value.data)
        builtEvent = builder.build()
        print("Sending %s %s..." % (eventType, builtEvent)),
        # and send it
        status = self.chat(lambda hal: hal.setProperty(
            eventTypeData['property'], 0, builtEvent)).status
        if status == 0:
            print("ok!")
        else:
            print("fail: %s" % status)
        del b


stopped = False
ENGINE_VALUE = 0
COOLANT_VALUE = 0
FUEL_VALUE = 0
INTAKE_PRESSURE_VALUE = 0
RPM_VALUE = 0
SPEED_VALUE = 0
INTAKE_TEMP_VALUE = 0
MAF_VALUE = 0
THROTTLE_VALUE = 0
lastTimestamp = 0


class Interfacetoandroid:
    def __init__(self, engine_load, coolant_temp, fuel_pressure,
                 intake_pressure, rpm, speed, intake_temp, maf, throttle_pos,
                 frame, sensortype):
        obdii = namedtuple(
            'obdii',
            ['name', 'data', 'timestamp', 'frame', 'id', 'sensortype'])
        self.lock = threading.Lock()
        self.lock.acquire()
        self.ENGINE_LOAD = obdii('Engine_Load', engine_load,
                                 time.mktime((datetime.datetime.now()).timetuple()), frame, DIAGNOSTIC_SENSOR_FLOAT_CALCULATED_ENGINE_LOAD,
                                 sensortype)
        self.COOLANT_TEMP = obdii('Coolant_Temp', coolant_temp,
                                  time.mktime((datetime.datetime.now()).timetuple()), frame,
                                  DIAGNOSTIC_SENSOR_FLOAT_ENGINE_COOLANT_TEMPERATURE, sensortype)
        self.FUEL_PRESSURE = obdii('Fuel_Pressure', fuel_pressure,
                                   time.mktime((datetime.datetime.now()).timetuple()), frame,
                                   DIAGNOSTIC_SENSOR_FLOAT_FUEL_PRESSURE, sensortype)
        self.INTAKE_PRESSURE = obdii('Intake_Pressure', intake_pressure,
                                     time.mktime((datetime.datetime.now()).timetuple()),
                                     frame, DIAGNOSTIC_SENSOR_FLOAT_INTAKE_MANIFOLD_ABSOLUTE_PRESSURE, sensortype)
        self.RPM = obdii('Rpm', rpm,
                         time.mktime((datetime.datetime.now()).timetuple()), frame, DIAGNOSTIC_SENSOR_FLOAT_ENGINE_RPM,
                         sensortype)
        self.SPEED = obdii('Speed', speed,
                           time.mktime((datetime.datetime.now()).timetuple()), frame, DIAGNOSTIC_SENSOR_FLOAT_VEHICLE_SPEED,
                           sensortype)
        #self.INTAKE_TEMP = obdii('Intake_Temp', intake_temp, time.mktime((datetime.datetime.now()).timetuple()) ,frame,4,'intValues')
        self.MAF = obdii('Maf', maf,
                         time.mktime((datetime.datetime.now()).timetuple()), frame, DIAGNOSTIC_SENSOR_FLOAT_MAF_AIR_FLOW_RATE,
                         sensortype)
        self.THROTTLE_POS = obdii('Throttle_Pos', throttle_pos,
                                  time.mktime((datetime.datetime.now()).timetuple()), frame,
                                  DIAGNOSTIC_SENSOR_FLOAT_THROTTLE_POSITION, sensortype)
        self.lock.release()
        #print(self.ENGINE_LOAD);

    def get_engine(self):
        return self.ENGINE_LOAD

    def get_coolant(self):
        return self.COOLANT_TEMP

    def get_fuel(self):
        return self.FUEL_PRESSURE

    def get_intake_pressure(self):
        return self.INTAKE_PRESSURE

    def get_rpm(self):
        return self.RPM

    def get_speed(self):
        return self.SPEED


#def get_intake_temp(self):
#       return self.INTAKE_TEMP

    def get_maf(self):
        return self.MAF

    def get_throttle_pos(self):
        return self.THROTTLE_POS

    def start(self):
        global stopped
        stopped = True

    def stop(self):
        global stopped
        stopped = False


def get_random_obdii():
    #print('obdii random value')
    global ENGINE_VALUE, COOLANT_VALUE, FUEL_VALUE, INTAKE_PRESSURE_VALUE, RPM_VALUE, SPEED_VALUE, INTAKE_TEMP_VALUE, MAF_VALUE, THROTTLE_VALUE
    ENGINE_VALUE = random.uniform(1, 10)
    COOLANT_VALUE = random.uniform(20, 40)
    FUEL_VALUE = random.uniform(40, 60)
    INTAKE_PRESSURE_VALUE = random.uniform(60, 80)
    RPM_VALUE = random.uniform(80, 100)
    SPEED_VALUE = random.uniform(100, 120)
    INTAKE_TEMP_VALUE = random.uniform(120, 140)
    MAF_VALUE = random.uniform(140, 160)
    THROTTLE_VALUE = random.uniform(160, 180)


def obd_loop(halWrapper):
    print('obd_loop')
    time.sleep(0.5)
    while True:
        print('In obd_loop')
        if stopped:
            print('obdii service started')
            for x in range(300):
                time.sleep(0.3)
                get_random_obdii()
                b = Interfacetoandroid(ENGINE_VALUE, COOLANT_VALUE, FUEL_VALUE,
                                       INTAKE_PRESSURE_VALUE, RPM_VALUE,
                                       SPEED_VALUE, INTAKE_TEMP_VALUE,
                                       MAF_VALUE, THROTTLE_VALUE, 'live',
                                       'floatValues')
                halWrapper.put_data(b)
        else:
            print('obdii service stopped')
        


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Diagnostic Events Injector')
    parser.add_argument('-s', action='store', dest='deviceid', default=None)

    args = parser.parse_args()

    halWrapper = DiagnosticHalWrapper(device=args.deviceid)
    t = threading.Thread(target=obd_loop, name="obdii", args=(halWrapper, ))
    t.setDaemon(True)
    t.start()
    while True:
        time.sleep(10)
        
