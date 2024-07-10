
QT       += core gui multimedia multimediawidgets sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets


TARGET      = videopanel
TEMPLATE    = app
DESTDIR     = $$PWD/bin
CONFIG      += warn_off

SOURCES     += main.cpp \
    cameraobject.cpp \
    camerasetdialog.cpp \
    qcamerauiwidget.cpp \
    userdatabase.cpp \
    ncnn-master/src/layer/absval.cpp \
    ncnn-master/src/layer/argmax.cpp \
    ncnn-master/src/layer/batchnorm.cpp \
    ncnn-master/src/layer/bias.cpp \
    ncnn-master/src/layer/binaryop.cpp \
    ncnn-master/src/layer/bnll.cpp \
    ncnn-master/src/layer/cast.cpp \
    ncnn-master/src/layer/clip.cpp \
    ncnn-master/src/layer/concat.cpp \
    ncnn-master/src/layer/convolution.cpp \
    ncnn-master/src/layer/convolutiondepthwise.cpp \
    ncnn-master/src/layer/crop.cpp \
    ncnn-master/src/layer/deconvolution.cpp \
    ncnn-master/src/layer/deconvolutiondepthwise.cpp \
    ncnn-master/src/layer/dequantize.cpp \
    ncnn-master/src/layer/detectionoutput.cpp \
    ncnn-master/src/layer/dropout.cpp \
    ncnn-master/src/layer/eltwise.cpp \
    ncnn-master/src/layer/elu.cpp \
    ncnn-master/src/layer/embed.cpp \
    ncnn-master/src/layer/exp.cpp \
    ncnn-master/src/layer/expanddims.cpp \
    ncnn-master/src/layer/flatten.cpp \
    ncnn-master/src/layer/hardsigmoid.cpp \
    ncnn-master/src/layer/hardswish.cpp \
    ncnn-master/src/layer/innerproduct.cpp \
    ncnn-master/src/layer/input.cpp \
    ncnn-master/src/layer/instancenorm.cpp \
    ncnn-master/src/layer/interp.cpp \
    ncnn-master/src/layer/log.cpp \
    ncnn-master/src/layer/lrn.cpp \
    ncnn-master/src/layer/lstm.cpp \
    ncnn-master/src/layer/memorydata.cpp \
    ncnn-master/src/layer/mvn.cpp \
    ncnn-master/src/layer/noop.cpp \
    ncnn-master/src/layer/normalize.cpp \
    ncnn-master/src/layer/packing.cpp \
    ncnn-master/src/layer/padding.cpp \
    ncnn-master/src/layer/permute.cpp \
    ncnn-master/src/layer/pixelshuffle.cpp \
    ncnn-master/src/layer/pooling.cpp \
    ncnn-master/src/layer/power.cpp \
    ncnn-master/src/layer/prelu.cpp \
    ncnn-master/src/layer/priorbox.cpp \
    ncnn-master/src/layer/proposal.cpp \
    ncnn-master/src/layer/psroipooling.cpp \
    ncnn-master/src/layer/quantize.cpp \
    ncnn-master/src/layer/reduction.cpp \
    ncnn-master/src/layer/relu.cpp \
    ncnn-master/src/layer/reorg.cpp \
    ncnn-master/src/layer/requantize.cpp \
    ncnn-master/src/layer/reshape.cpp \
    ncnn-master/src/layer/rnn.cpp \
    ncnn-master/src/layer/roialign.cpp \
    ncnn-master/src/layer/roipooling.cpp \
    ncnn-master/src/layer/scale.cpp \
    ncnn-master/src/layer/selu.cpp \
    ncnn-master/src/layer/shufflechannel.cpp \
    ncnn-master/src/layer/sigmoid.cpp \
    ncnn-master/src/layer/slice.cpp \
    ncnn-master/src/layer/softmax.cpp \
    ncnn-master/src/layer/split.cpp \
    ncnn-master/src/layer/spp.cpp \
    ncnn-master/src/layer/squeeze.cpp \
    ncnn-master/src/layer/tanh.cpp \
    ncnn-master/src/layer/threshold.cpp \
    ncnn-master/src/layer/tile.cpp \
    ncnn-master/src/layer/unaryop.cpp \
    ncnn-master/src/layer/yolodetectionoutput.cpp \
    ncnn-master/src/layer/yolov3detectionoutput.cpp \
    ncnn-master/src/allocator.cpp \
    ncnn-master/src/benchmark.cpp \
    ncnn-master/src/blob.cpp \
    ncnn-master/src/command.cpp \
    ncnn-master/src/cpu.cpp \
    ncnn-master/src/datareader.cpp \
    ncnn-master/src/gpu.cpp \
    ncnn-master/src/layer.cpp \
    ncnn-master/src/mat.cpp \
    ncnn-master/src/mat_pixel.cpp \
    ncnn-master/src/mat_pixel_android.cpp \
    ncnn-master/src/mat_pixel_resize.cpp \
    ncnn-master/src/mat_pixel_rotate.cpp \
    ncnn-master/src/modelbin.cpp \
    ncnn-master/src/net.cpp \
    ncnn-master/src/opencv.cpp \
    ncnn-master/src/option.cpp \
    ncnn-master/src/paramdict.cpp \
    ncnn-master/src/pipeline.cpp \
    yolo/yolo-fastest.cpp \
    target.cpp \
    MyLabel.cpp \
    LeptonThread.cpp




SOURCES     += frmvideopanel.cpp
SOURCES     += videopanel.cpp

HEADERS     += frmvideopanel.h \
    cameraobject.h \
    camerasetdialog.h \
    qcamerauiwidget.h \
    userdatabase.h \
    ncnn-master/src/layer/absval.h \
    ncnn-master/src/layer/argmax.h \
    ncnn-master/src/layer/batchnorm.h \
    ncnn-master/src/layer/bias.h \
    ncnn-master/src/layer/binaryop.h \
    ncnn-master/src/layer/bnll.h \
    ncnn-master/src/layer/cast.h \
    ncnn-master/src/layer/clip.h \
    ncnn-master/src/layer/concat.h \
    ncnn-master/src/layer/convolution.h \
    ncnn-master/src/layer/convolutiondepthwise.h \
    ncnn-master/src/layer/crop.h \
    ncnn-master/src/layer/deconvolution.h \
    ncnn-master/src/layer/deconvolutiondepthwise.h \
    ncnn-master/src/layer/dequantize.h \
    ncnn-master/src/layer/detectionoutput.h \
    ncnn-master/src/layer/dropout.h \
    ncnn-master/src/layer/eltwise.h \
    ncnn-master/src/layer/elu.h \
    ncnn-master/src/layer/embed.h \
    ncnn-master/src/layer/exp.h \
    ncnn-master/src/layer/expanddims.h \
    ncnn-master/src/layer/flatten.h \
    ncnn-master/src/layer/hardsigmoid.h \
    ncnn-master/src/layer/hardswish.h \
    ncnn-master/src/layer/innerproduct.h \
    ncnn-master/src/layer/input.h \
    ncnn-master/src/layer/instancenorm.h \
    ncnn-master/src/layer/interp.h \
    ncnn-master/src/layer/log.h \
    ncnn-master/src/layer/lrn.h \
    ncnn-master/src/layer/lstm.h \
    ncnn-master/src/layer/memorydata.h \
    ncnn-master/src/layer/mvn.h \
    ncnn-master/src/layer/noop.h \
    ncnn-master/src/layer/normalize.h \
    ncnn-master/src/layer/packing.h \
    ncnn-master/src/layer/padding.h \
    ncnn-master/src/layer/permute.h \
    ncnn-master/src/layer/pixelshuffle.h \
    ncnn-master/src/layer/pooling.h \
    ncnn-master/src/layer/power.h \
    ncnn-master/src/layer/prelu.h \
    ncnn-master/src/layer/priorbox.h \
    ncnn-master/src/layer/proposal.h \
    ncnn-master/src/layer/psroipooling.h \
    ncnn-master/src/layer/quantize.h \
    ncnn-master/src/layer/reduction.h \
    ncnn-master/src/layer/relu.h \
    ncnn-master/src/layer/reorg.h \
    ncnn-master/src/layer/requantize.h \
    ncnn-master/src/layer/reshape.h \
    ncnn-master/src/layer/rnn.h \
    ncnn-master/src/layer/roialign.h \
    ncnn-master/src/layer/roipooling.h \
    ncnn-master/src/layer/scale.h \
    ncnn-master/src/layer/selu.h \
    ncnn-master/src/layer/shufflechannel.h \
    ncnn-master/src/layer/sigmoid.h \
    ncnn-master/src/layer/slice.h \
    ncnn-master/src/layer/softmax.h \
    ncnn-master/src/layer/split.h \
    ncnn-master/src/layer/spp.h \
    ncnn-master/src/layer/squeeze.h \
    ncnn-master/src/layer/tanh.h \
    ncnn-master/src/layer/threshold.h \
    ncnn-master/src/layer/tile.h \
    ncnn-master/src/layer/unaryop.h \
    ncnn-master/src/layer/yolodetectionoutput.h \
    ncnn-master/src/layer/yolov3detectionoutput.h \
    ncnn-master/src/allocator.h \
    ncnn-master/src/benchmark.h \
    ncnn-master/src/blob.h \
    ncnn-master/src/command.h \
    ncnn-master/src/convolution.h \
    ncnn-master/src/convolutiondepthwise.h \
    ncnn-master/src/cpu.h \
    ncnn-master/src/datareader.h \
    ncnn-master/src/gpu.h \
    ncnn-master/src/layer.h \
    ncnn-master/src/layer_declaration.h \
    ncnn-master/src/layer_registry.h \
    ncnn-master/src/layer_type.h \
    ncnn-master/src/layer_type_enum.h \
    ncnn-master/src/mat.h \
    ncnn-master/src/modelbin.h \
    ncnn-master/src/net.h \
    ncnn-master/src/opencv.h \
    ncnn-master/src/option.h \
    ncnn-master/src/paramdict.h \
    ncnn-master/src/pipeline.h \
    ncnn-master/src/platform.h \
    ncnn-master/src/relu.h \
    yolo/yolo-fastest.h \
    target.h \
    MyLabel.h \
    ui_mainwindow.h \
    LeptonThread.h



HEADERS     += videopanel.h

FORMS       += frmvideopanel.ui \
    camerasetdialog.ui




SUBDIRS += \
    videopanel.pro

INCLUDEPATH += ./ncnn-master/src
INCLUDEPATH += ./ncnn-master/src/layer

INCLUDEPATH += /home/lhy/opencv-3.4.6/install2/include \
               /home/lhy/opencv-3.4.6/install2/include/opencv \
               /home/lhy/opencv-3.4.6/install2/include/opencv2

LIBS += /home/lhy/opencv-3.4.6/install2/lib/libopencv_*
