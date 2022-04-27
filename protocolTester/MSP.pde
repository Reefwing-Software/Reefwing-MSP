/******************************************************************
 Multiwii Serial Protocol
 ******************************************************************/
 
private static final String MSP_HEADER = "$M<";  //  MSP Version 1.0

// Request ID's below 100 are Betaflight specific

private static final int
  MSP_API_VERSION          =1,    
  MSP_FC_VARIANT           =2,    
  MSP_FC_VERSION           =3,    
  MSP_BOARD_INFO           =4,    
  MSP_BUILD_INFO           =5,    
  MSP_NAME                 =10,   
  MSP_SET_NAME             =11,   
  MSP_IDENT                =100,
  MSP_STATUS               =101,
  MSP_RAW_IMU              =102,
  MSP_SERVO                =103,
  MSP_MOTOR                =104,
  MSP_RC                   =105,
  MSP_RAW_GPS              =106,
  MSP_COMP_GPS             =107,
  MSP_ATTITUDE             =108,
  MSP_ALTITUDE             =109,
  MSP_ANALOG               =110,
  MSP_RC_TUNING            =111,
  MSP_PID                  =112,
  MSP_BOX                  =113,
  MSP_MISC                 =114,
  MSP_MOTOR_PINS           =115,
  MSP_BOXNAMES             =116,
  MSP_PIDNAMES             =117,
  MSP_SERVO_CONF           =120,
    
  
  MSP_SET_RAW_RC           =200,
  MSP_SET_RAW_GPS          =201,
  MSP_SET_PID              =202,
  MSP_SET_BOX              =203,
  MSP_SET_RC_TUNING        =204,
  MSP_ACC_CALIBRATION      =205,
  MSP_MAG_CALIBRATION      =206,
  MSP_SET_MISC             =207,
  MSP_RESET_CONF           =208,
  MSP_SELECT_SETTING       =210,
  MSP_SET_HEAD             =211, // Not used
  MSP_SET_SERVO_CONF       =212,
  MSP_SET_MOTOR            =214,
  
  
  MSP_BIND                 =241,

  MSP_EEPROM_WRITE         =250,
  
  MSP_DEBUGMSG             =253,
  MSP_DEBUG                =254
;

public enum CommandID {
  MSP_API_VERSION(1),    
  MSP_FC_VARIANT(2),    
  MSP_FC_VERSION(3),    
  MSP_BOARD_INFO(4),    
  MSP_BUILD_INFO(5),    
  MSP_NAME(10),   
  MSP_SET_NAME(11), 
  MSP_IDENT(100),
  MSP_STATUS(101),
  MSP_RAW_IMU(102),
  MSP_SERVO(103),
  MSP_MOTOR(104),
  MSP_RC(105),
  MSP_RAW_GPS(106),
  MSP_COMP_GPS(107),
  MSP_ATTITUDE(108),
  MSP_ALTITUDE(109),
  MSP_ANALOG(110),
  MSP_RC_TUNING(111),
  MSP_PID(112),
  MSP_BOX(113),
  MSP_MISC(114),
  MSP_MOTOR_PINS(115),
  MSP_BOXNAMES(116),
  MSP_PIDNAMES(117),
  MSP_SERVO_CONF(120)
  ;
  
  public final int code;
  
  private CommandID(int code) {
    this.code = code;
  }
  
  public static CommandID valueOfCode(int code) {
    for (CommandID c : values()) {
      if (c.code == code) {
        return c;
      }
    }
    return null;
  }
  
};

/******************************************************************
 Alias for multiTypes (MultiWiiConf)
 ******************************************************************/

final int TRI           =1;
final int QUADP         =2;
final int QUADX         =3;
final int BI            =4;
final int GIMBAL        =5;
final int Y6            =6;
final int HEX6          =7;
final int FLYING_WING   =8;
final int Y4            =9;
final int HEX6X         =10;
final int OCTOX8        =11;
final int OCTOFLATX     =12;
final int OCTOFLATP     =13;
final int AIRPLANE      =14;
final int HELI_120_CCPM =15;
final int HELI_90_DEG   =16;
final int VTAIL4        =17;
final int HEX6H         =18;
final int PPM_TO_SERVO  =19;
final int DUALCOPTER    =20;
final int SINGLECOPTER  =21;

/******************************************************************
 Serial Message State Machine & Globals (MultiWiiConf)
 ******************************************************************/

public static final int
  IDLE = 0,
  HEADER_START = 1,
  HEADER_M = 2,
  HEADER_ARROW = 3,
  HEADER_SIZE = 4,
  HEADER_CMD = 5,
  HEADER_ERR = 6
;

int c_state = IDLE;
boolean err_rcvd = false;

byte checksum=0;
byte cmd;
int offset=0, dataSize=0;
byte[] inBuf = new byte[256];


int p;
int read32() {return (inBuf[p++]&0xff) + ((inBuf[p++]&0xff)<<8) + ((inBuf[p++]&0xff)<<16) + ((inBuf[p++]&0xff)<<24); }
int read16() {return (inBuf[p++]&0xff) + ((inBuf[p++])<<8); }
int read8()  {return  inBuf[p++]&0xff;}
        
/******************************************************************
 MSP Protocol Functions (MultiWiiConf)
 ******************************************************************/

//send msp without payload
private List<Byte> requestMSP(int msp) {
  return  requestMSP( msp, null);
}

//send multiple msp without payload
private List<Byte> requestMSP (int[] msps) {
  List<Byte> s = new LinkedList<Byte>();
  for (int m : msps) {
    s.addAll(requestMSP(m, null));
  }
  return s;
}

//send msp with payload
private List<Byte> requestMSP (int msp, Character[] payload) {
  if(msp < 0) {
   return null;
  }
  List<Byte> bf = new LinkedList<Byte>();
  for (byte c : MSP_HEADER.getBytes()) {
    bf.add( c );
  }
  
  byte checksum=0;
  byte pl_size = (byte)((payload != null ? int(payload.length) : 0)&0xFF);
  bf.add(pl_size);
  checksum ^= (pl_size&0xFF);
  
  bf.add((byte)(msp & 0xFF));
  checksum ^= (msp&0xFF);
  
  if (payload != null) {
    for (char c :payload){
      bf.add((byte)(c&0xFF));
      checksum ^= (c&0xFF);
    }
  }
  bf.add(checksum);
  return (bf);
}

void sendRequestMSP(List<Byte> msp) {
  byte[] arr = new byte[msp.size()];
  int i = 0;
  for (byte b: msp) {
    arr[i++] = b;
  }
  serialPort.write(arr); // send the complete byte sequence in one go
}

/******************************************************************
 Evaluate MSP Responses (MultiWiiConf)
 ******************************************************************/

public void evaluateCommand(byte cmd, int dataSize) {
  int i;
  int icmd = (int)(cmd&0xFF);
  int variant;
  
  switch(icmd) {
    case MSP_IDENT:
      version = read8();
      multiType = read8();
      read8(); // MSP version - not used
      multiCapability = read32();// capability
      logConsole("MSP_IDENT response received");
      logConsole("Version: " + version + " multitype: " + multiType + " multicapability: " + multiCapability);
      break;
    case MSP_STATUS:
      cycleTime = read16();
      i2cError = read16();
      present = read16();
      mode = read32();  //  a bit variable to indicate which BOX are active, the bit position depends on the BOX which are configured
      configSetting = read8();
      break;
    case MSP_RAW_IMU:
      ax = read16(); ay = read16(); az = read16();
      gx = read16(); gy = read16(); gz = read16();
      mx = read16(); my = read16(); mz = read16();
      break;
    case MSP_FC_VARIANT:
      logConsole("MSP_FC_VARIANT response received");
      variant = read8();
      logConsole("Variant = " + variant);
      break;
    default:
      logConsole("Unhandled MSP MSG Response: " + icmd);
  }
}
