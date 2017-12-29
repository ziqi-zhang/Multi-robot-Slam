//You need to program this file.

#include "../NoEdit/ProcessorMulti_Processor_Core_PrivFunc.h"
#include "Communication/tcpclient.h"



//*******************Please add static libraries in .pro file*******************
//e.g. unix:LIBS += ... or win32:LIBS += ...

bool DECOFUNC(setParamsVarsOpenNode)(QString qstrConfigName, QString qstrNodeType, QString qstrNodeClass, QString qstrNodeName, void * paramsPtr, void * varsPtr)
{
	XMLDomInterface xmlloader(qstrConfigName,qstrNodeType,qstrNodeClass,qstrNodeName);
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	/*======Please Program below======*/
	/*
	Function: open node.
	Procedure:
	1: load parameters (params). [GetParamValue(xmlloader,params,tag); GetEnumParamValue(xmlloader,params,tag); GetUEnumParamValue(xmlloader,params,tag)]
	2: initialize variables (vars).
	3: If everything is OK, return 1 for successful opening and vice versa.
	*/
	
	return 1;
}

bool DECOFUNC(handleVarsCloseNode)(void * paramsPtr, void * varsPtr)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	/*======Please Program below======*/
	/*
	Function: close node.
	Procedure:
	1: handle/close variables (vars).
	2: If everything is OK, return 1 for successful closing and vice versa.
	*/
	
	return 1;
}

void DECOFUNC(getInternalTrigger)(void * paramsPtr, void * varsPtr, QObject * & internalTrigger, QString & internalTriggerSignal)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	internalTrigger=NULL;
	internalTriggerSignal=QString();
	/*======Occasionally Program above======*/
	/*
	Function: get internal trigger [defined in vars] for node.
	You need to program here when you need internal trigger (internalTrigger + internalTriggerSignal) for node.
	E.g.
	internalTrigger=&(vars->trigger);
	internalTriggerSignal=QString(SIGNAL(triggerSignal()));
	*/
}

void DECOFUNC(initializeOutputData)(void * paramsPtr, void * varsPtr, boost::shared_ptr<void> & outputDataPtr)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	outputDataPtr=boost::shared_ptr<void>(new SourceDrainMono_Sensor_EncoderIMU_Order_InputData());
	/*======Occasionally Program below/above======*/
	/*
	Function: initial output data.
	You need to program here when you need to manually initialize output data.
	*/
	
}

void DECOFUNC(getMultiInputDataSize)(void * paramsPtr, void * varsPtr, QList<int> & inputDataSize)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	inputDataSize=QList<int>();
	/*======Please Program above======*/
	/*
	Function: get input data size to be grabbed from buffer.
	Rules:
	inputDataSize=0: grab and remove all data from buffer.
	inputDataSize>0: grab inputDataSize latest data from buffer.
	inputDataSize<0: grab and remove inputDataSize ancient data from buffer.
	E.g.
	inputDataSize=QList<int>()<<0<<1<<-1...;
	*/
}

void calFromDist(QVector<SensorTimer_Sensor_URG_Data *> &inputdata_1,
                  short &speed, short &steer,ProcessorMulti_Processor_Core_Vars* varPtr){
    int upper = 100;
    int timeStamp = inputdata_1.front()->timestamp;
    double ave_dist = 0;
    for( int i=0; i<upper; i++ ){
        double dis = inputdata_1.front()->data[i]/(double)100; // get the ray dis
        double  angle = i*M_PI/360;
        ave_dist += dis*cos(angle);
    }
    double error = ave_dist - 100;
    if(varPtr->bef_count < 10){
        varPtr->Error_bef[varPtr->bef_count] = error;
        varPtr->bef_count++;
        steer = 50 + error*10;
    }
    else
    {
        double Div = (error - varPtr->Error_bef[8])/2;
        double Intgrt = varPtr->Error_bef[0];
        for(int i =1;i<10;i++){
            Intgrt += varPtr->Error_bef[i];
            varPtr->Error_bef[i-1] = varPtr->Error_bef[i];
        }
        varPtr->Error_bef[9] = error;
        steer = 50 + 5*error + 1*Intgrt + 1*Div;
    }
    if(steer > 400) steer =400;
    if(steer < -400) steer =-400;
    speed = 90;           // [-180, 180]


    //qDebug() << speed << steer << ave_dist;
    // Show RGB image && compass
//    double ori = - ((double)steer / 400.0) * (M_PI / 2.0);
//    cv::Mat img;
//    inputdata_2.front()->cvColorImg.copyTo(img);
//    cv::flip(img, img, 1);
//    cv::Point compass = cv::Point(100, 100);
//    cv::circle(img, compass, 80, cv::Scalar(0,255,0), 1, CV_AA);
//    cv::line(img, compass,
//             cv::Point(compass.x - sin(ori) * 80,
//                       compass.y - cos(ori) * 80),
//             cv::Scalar(0,255,0), 3, CV_AA);


//    char str[200];
//    sprintf(str, "dist %.2lf", ave_dist);
//    cv::putText(img, str, cv::Point(200,100), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255,0,0));


//    cv::imshow("color", img);
}

//Input Port #0: Buffer_Size = 10, Params_Type = SourceDrainMono_Sensor_EncoderIMU_Params, Data_Type = SourceDrainMono_Sensor_EncoderIMU_Data
//Input Port #1: Buffer_Size = 10, Params_Type = SensorTimer_Sensor_URG_Params, Data_Type = SensorTimer_Sensor_URG_Data
//Input Port #2: Buffer_Size = 10, Params_Type = SensorTimer_Sensor_xtion_Params, Data_Type = SensorTimer_Sensor_xtion_Data
bool DECOFUNC(processMultiInputData)(void * paramsPtr, void * varsPtr, QVector<QVector<void *> > inputParams, QVector<QVector<void *> > inputData, void * outputData, QList<int> & outputPortIndex)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	QVector<SourceDrainMono_Sensor_EncoderIMU_Params *> inputparams_0; copyQVector(inputparams_0,inputParams[0]);
	QVector<SensorTimer_Sensor_URG_Params *> inputparams_1; copyQVector(inputparams_1,inputParams[1]);
	QVector<SensorTimer_Sensor_xtion_Params *> inputparams_2; copyQVector(inputparams_2,inputParams[2]);
	QVector<SourceDrainMono_Sensor_EncoderIMU_Data *> inputdata_0; copyQVector(inputdata_0,inputData[0]);
	QVector<SensorTimer_Sensor_URG_Data *> inputdata_1; copyQVector(inputdata_1,inputData[1]);
	QVector<SensorTimer_Sensor_xtion_Data *> inputdata_2; copyQVector(inputdata_2,inputData[2]);
	SourceDrainMono_Sensor_EncoderIMU_Order_InputData * outputdata=(SourceDrainMono_Sensor_EncoderIMU_Order_InputData *)outputData;
	outputPortIndex=QList<int>();
	if(inputdata_0.size()==0){return 0;}
	if(inputdata_1.size()==0){return 0;}
	if(inputdata_2.size()==0){return 0;}




	/*======Please Program below======*/
	/*
	Step 1: process inputdata_index, then store it into outputdata.
	Step 2 [optional]: determine the outputPortIndex. (if not, outputdata will be sent by all ports)
	E.g. outputPortIndex=QList<int>()<<(outportindex1)<<(outportindex2)...
	*/

    short steer = 0;
    short speed = 200;
    long long timeStamp = inputdata_1.front()->timestamp;
    qDebug() << " pos = " << inputdata_0.front()->x <<"  "<< inputdata_0.front()->y;
    float x = inputdata_0.front()->x;
    float y = inputdata_0.front()->y;
    float ori = inputdata_0.front()->orientation;
    if( vars->cnt>=3 ){
        vars->cnt =0 ;
    Network network;
    TcpClient client(&network);
    client.SendMessageToHost(x, y, ori, inputdata_1.front());
    }
    else
        vars->cnt++;
    calFromDist(inputdata_1, speed, steer,vars);



    //--------------------------------------------
    if (speed > 180) speed = 180;
    if (speed < -180) speed = -180;
    char dataput[20];
    dataput[0] = 0xF8;
    dataput[1] = 4;
    *(short*)&dataput[2] = (short)steer;
    *(short*)&dataput[4] = (short)speed;
    dataput[6] = 0x8F;
    outputdata->datagram = QByteArray(dataput, 7);
	return 1;
}

