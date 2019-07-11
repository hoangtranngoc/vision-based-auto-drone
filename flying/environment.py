import sys
if "E:\\git\\AirSim\\PythonClient" not in sys.path:
    sys.path.append("E:\\git\\AirSim\\PythonClient")
    print(sys.path)
import airsim
import random


rain_values = [0, 0.5, 1]
snow_values = [0, 0.5, 1]
fog_values = [0, 0.25, 0.5, 0.75, 1]
mapleleaf_values = [0, 0, 0, 0, 0.25, 0.5]
dust_values = [0, 0.1, 0.25, 0.5, 0.75, 1]

class Environment:
        
    def __init__(self, client):
        self.client = client
      
    def set_weather(self, rain=0, snow=0, fog=0, mapleleaf=0, dust=0, isRandom=False):
        self.client.simEnableWeather(True)   
        if isRandom:
            r = random.choice(rain_values)
            s = random.choice(snow_values)
            f = random.choice(fog_values)
            l = random.choice(mapleleaf_values)
            d = random.choice(dust_values)   
        else:
            r = rain
            s = snow
            f = fog
            l = mapleleaf
            d = dust

        print ("set weather r{}, s{}, f{}, l{}, d{}".format(r,s,f,l,d))
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Rain, r)
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Snow, s)
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Fog, f)
        self.client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, l)
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Dust, d)