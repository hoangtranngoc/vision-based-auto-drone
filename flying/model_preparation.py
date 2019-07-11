import sys
if "E:\\git\\keras-resnet" not in sys.path:
    sys.path.append("E:\\git\\keras-resnet")
    print(sys.path)
import resnet

from keras.layers import GlobalAveragePooling2D, Dense
from keras.models import load_model, Model 
from keras.applications.mobilenetv2 import MobileNetV2
from keras.applications.densenet import DenseNet121
from keras.applications.nasnet import NASNetMobile
import keras.applications

  
# model_id: 0 - Resnet18, 1 - MobileNetV2, 2 - DenseNet121, 3 - NasNetMobile
def build_model(voa_model_path, pitch_model_path, model_id):        
    voa_model = load_model(voa_model_path)
    cam_pitch_model = load_model(pitch_model_path)

    if model_id == 0:
        model = build_ResNet18_model(multi_output = 4, nb_classes = 3)
    elif model_id == 1:
        model = build_MobileNetV2_model(multi_output = 4, nb_classes = 3)
    elif model_id == 2:
        model = build_DenseNet121_model(multi_output = 4, nb_classes = 3)
    elif model_id == 3:
        model = build_NasnetMobile_model(multi_output = 4, nb_classes = 3)
    else: 
        print("not supported yet")

    w1 = voa_model.get_weights()
    w2 = cam_pitch_model.get_weights()
    w = w1 + w2[-2:]

    model.set_weights(w)  
    return model


def build_ResNet18_model(multi_output = 4, nb_classes = 3):
    model = resnet.ResnetBuilder.build_resnet_18_uav((3, 512, 512), nb_classes, multi_output)
    return model

def build_MobileNetV2_model(multi_output = 4, nb_classes = 3):
    net = MobileNetV2(input_shape=(512, 512, 3), include_top=None, weights=None, classes=nb_classes)
    x = net.output

    outputs = []

    for i in range(multi_output):
        pool = GlobalAveragePooling2D()(x)
        dense = Dense(nb_classes, activation='softmax', use_bias=True)(pool)
        outputs.append(dense)

    model = Model(net.inputs, outputs=outputs, name='mobilenetv2')

    return model


def build_DenseNet121_model(multi_output = 4, nb_classes = 3):
    net = DenseNet121(input_shape=(512, 512, 3), include_top=False, weights=None, classes=nb_classes)
    x = net.output

    outputs = []

    for i in range(multi_output):
        pool = GlobalAveragePooling2D()(x)
        dense = Dense(nb_classes, activation='softmax')(pool)
        outputs.append(dense)

    model = Model(net.inputs, outputs=outputs, name='densenet')
    return model


def build_NasnetMobile_model(multi_output = 4, nb_classes = 3):
    net = NASNetMobile(input_shape=(512, 512, 3), include_top=False, weights=None, classes=nb_classes)
    x = net.output

    outputs = []

    for i in range(multi_output):
        avg = GlobalAveragePooling2D()(x)
        dense = Dense(nb_classes, activation='softmax')(avg)
        outputs.append(dense)

    model = Model(net.inputs, outputs=outputs, name='nasnet')
    return model
    