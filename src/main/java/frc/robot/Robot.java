package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
 
  //region
  Joystick Controle_0 = new Joystick(0);

  CANSparkMax RotacaoFD = new CANSparkMax(00, MotorType.kBrushed);//TODO TROCAR OS IDS DOS SPARKS PARA OS USADOS NO SEU SISTEMA
  CANSparkMax RotacaoFE = new CANSparkMax(00, MotorType.kBrushed);
  CANSparkMax RotacaoTE = new CANSparkMax(00, MotorType.kBrushed);
  CANSparkMax RotacaoTD = new CANSparkMax(00, MotorType.kBrushed);
 
  CANSparkMax TracaoFD = new CANSparkMax(00, MotorType.kBrushless);
  CANSparkMax TracaoFE = new CANSparkMax(00, MotorType.kBrushless);
  CANSparkMax TracaoTE = new CANSparkMax(00, MotorType.kBrushless);
  CANSparkMax TracaoTD = new CANSparkMax(00, MotorType.kBrushless);

  DutyCycleEncoder EncoderFD = new DutyCycleEncoder(1); //Portas DIO dos encoders
  DutyCycleEncoder EncoderFE = new DutyCycleEncoder(2); //
  DutyCycleEncoder EncoderTE = new DutyCycleEncoder(3); //
  DutyCycleEncoder EncoderTD = new DutyCycleEncoder(4); //

  PIDController Pid_FD = new PIDController(0, 0, 0);//Trocar as constantes para melhor controlar o seu sistema.
  PIDController Pid_FE = new PIDController(0, 0, 0);//Geralmente só o kp já é suficiente
  PIDController Pid_TE = new PIDController(0, 0, 0);//
  PIDController Pid_TD = new PIDController(0, 0, 0);//

  double EncoderFDCorrigido;
  double EncoderFECorrigido;  
  double EncoderTECorrigido;
  double EncoderTDCorrigido;

  double UltimoAngulo_FD;
  double UltimoAngulo_FE;
  double UltimoAngulo_TE;
  double UltimoAngulo_TD;

  double VelAntigaFD;
  double VelAntigaFE;
  double VelAntigaTE;
  double VelAntigaTD;

  double Joystick_X;
  double Joystick_Y;
  double Joystick_R;

  
  double TempoAtual;
  double DiferencaTempoAtual;

  //endregion
  
  public double NormalizarAngulo(double AnguloInput){ //Corrige um ângulo de qualquer valor em um ângulo equivalente situado entre 180° e -180° 
    // Ex:.
    //   190 -> -170
    //   365 -> 5
    //  -200 -> 160 

    //A wpilib até tem a função Mathutil.anglemodulus() que faz a mesma coisa, mas essa só funciona com ângulo em radianos
      
    //Fonte: https://stackoverflow.com/a/2323034

    //Reduz o ângulo
    AnguloInput = AnguloInput % 360;
   
    //Força o ângulo a ser o resto positivo, de modo que 0 <= ângulo < 360
    AnguloInput = (AnguloInput+360) % 360;
   
    if(AnguloInput>180){
      return  (AnguloInput - 360);
    }
    else{
      return AnguloInput;
    }
    
  }

  public void MovimentarSwerve(double Input_X, double Input_Y, double Input_R){
    final double TaxaMax = 1.3; //Limite de mudança por segundo do output das rodas de tração
    
    boolean MotorFDInvertido = false;    
    boolean MotorFEInvertido = false;  
    boolean MotorTEInvertido = false;  
    boolean MotorTDInvertido = false;  

    

    Input_R = Input_R * 0.71;// O fator de conversão de 0.71 corresponde ao seno/cosseno de 45°, o que dá o componente correto na vertical/horizontal a partir da magnitude de Input_R.
                             // Vale notar que isso assume uma disposição de módulos quadrada, com o centro de massa idealmente no centro geométrico dos módulos.

    double CompVertFD = Input_Y - Input_R; // Acha os componentes verticais do vetor de cada módulo.
    double CompVertFE = Input_Y + Input_R; //
    double CompVertTE = Input_Y + Input_R; //
    double CompVertTD = Input_Y - Input_R; // 

    double CompHorzFD = Input_X + Input_R; // Acha os componentes horizontais do vetor de cada módulo.
    double CompHorzFE = Input_X + Input_R; //
    double CompHorzTE = Input_X - Input_R; //
    double CompHorzTD = Input_X - Input_R; //

    //CÁLCULOS DA ROTAÇÃO

    double AnguloFD = Math.atan2(CompHorzFD, CompVertFD)*57.3; // Descobre a direção para qual cada vetor está apontado a partir de seus componentes.
    double AnguloFE = Math.atan2(CompHorzFE, CompVertFE)*57.3; // O resultado disso é um ângulo de -180 a +180, com 0 sendo a frente/cima.
    double AnguloTE = Math.atan2(CompHorzTE, CompVertTE)*57.3; //
    double AnguloTD = Math.atan2(CompHorzTD, CompVertTD)*57.3; //



    if((Input_X==0) && (Input_Y==0) && (Input_R==0)) { //Se o controle estiver na zona morta, substitui
      AnguloFD = UltimoAngulo_FD;                      // o ângulo desejado pelo último ângulo calculado 
      AnguloFE = UltimoAngulo_FE;                      // enquanto o controle ainda estava sendo utilizado.
      AnguloTE = UltimoAngulo_TE;
      AnguloTD = UltimoAngulo_TD;
    } 

    UltimoAngulo_FD = AnguloFD;//Armazena o último ângulo calculado para ser utilizado no próximo loop,
    UltimoAngulo_FE = AnguloFE;// caso o controle esteja na zona morta.
    UltimoAngulo_TE = AnguloTE;//
    UltimoAngulo_TD = AnguloTD;//



    if(Math.abs(AnguloFD)>90){                   //Confere se o ângulo encontrado vai passar do limite de +/-90° do módulo.
      AnguloFD = NormalizarAngulo(AnguloFD+180); //Se passar, troca o ângulo por um oposto a ele mesmo e marca o módulo como "invertido".
      MotorFDInvertido = true;                   //
    }                                            //
    // else{                                        //Eu acho que n precisa dessas linhas que tão comentadas
    //   MotorFDInvertido = false;                  //talvez depois eu apague
    // }                                            //se puder testar se faz diferença eu agradeço
    //                                           //
    if(Math.abs(AnguloFE)>90){                   //
      AnguloFE = NormalizarAngulo(AnguloFE+180); //
      MotorFEInvertido = true;                   //
    }                                            //
    // else{                                        //
    //   MotorFEInvertido = false;                  //
    // }                                            //
    //                                           //
    if(Math.abs(AnguloTE)>90){                   //
      AnguloTE = NormalizarAngulo(AnguloTE+180); //
      MotorTEInvertido = true;                   //
    }                                            //
    // else{                                        //
    //   MotorTEInvertido = false;                  //
    // }                                            //
    //                                           //
    if(Math.abs(AnguloTD)>90){                   //
      AnguloTD = NormalizarAngulo(AnguloTD+180); //
      MotorTDInvertido = true;                   //
    }                                            //
    // else{                                        //
    //   MotorTDInvertido = false;                  //
    // }                                            //



    double direcaoFD = Pid_FD.calculate(EncoderFDCorrigido,AnguloFD);//Usa o controlador PID para encontrar
    double direcaoFE = Pid_FE.calculate(EncoderFECorrigido,AnguloFE);// uma velocidade de motor que leve a roda à direção desejada.
    double direcaoTE = Pid_TE.calculate(EncoderTECorrigido,AnguloTE);//  
    double direcaoTD = Pid_TD.calculate(EncoderTDCorrigido,AnguloTD);//  

    direcaoFD = MathUtil.clamp(direcaoFD, -0.2, 0.2);//Limita o valor de velocidade que vai para o motor de rotação.
    direcaoFE = MathUtil.clamp(direcaoFE, -0.2, 0.2);//Isso ajuda a limitar a corrente dos motores para evitar queda de tensão da bateria.
    direcaoTE = MathUtil.clamp(direcaoTE, -0.2, 0.2);//
    direcaoTD = MathUtil.clamp(direcaoTD, -0.2, 0.2);//           

    RotacaoFD.set(direcaoFD);//Manda o valor calculado para o motor.
    RotacaoFE.set(direcaoFE);//    
    RotacaoTE.set(direcaoTE);//    
    RotacaoTD.set(direcaoTD);//    


    //CÁLCULOS DA TRAÇÃO

    double VelFD = Math.hypot(CompVertFD, CompHorzFD);//Calcula o valor da velocidade como sendo o comprimento do vetor calculado inicialmente.
    double VelFE = Math.hypot(CompVertFE, CompHorzFE);//
    double VelTE = Math.hypot(CompVertTE, CompHorzTE);//
    double VelTD = Math.hypot(CompVertTD, CompHorzTD);//


    
    double[] velocidades = {VelFD, VelFE, VelTE, VelTD};// Descobre a maior velocidade.
    double MaiorVelocidade = 1;                         //
    for (double i: velocidades){                        //
      if(i>MaiorVelocidade){MaiorVelocidade=i;}         //
    }                                                   //

    VelFD = (VelFD/MaiorVelocidade);//Divide todas as velocidades pela maior, ou por 1, caso todas estejam abaixo de 1.
    VelFE = (VelFE/MaiorVelocidade);//Isso faz com que a maior velocidade sempre esteja limitada a 1, e faz com que todas as outras se ajustem proporcionalmente.
    VelTE = (VelTE/MaiorVelocidade);//
    VelTD = (VelTD/MaiorVelocidade);//



    VelFD = MotorFDInvertido ? VelFD : -VelFD; //Inverte a velocidade que vai pra os motores de tração, caso
    VelFE = MotorFEInvertido ? VelFE : -VelFE; // o motor esteja marcado como "invertido".
    VelTE = MotorTEInvertido ? VelTE : -VelTE; //
    VelTD = MotorTDInvertido ? VelTD : -VelTD; //

    
    
    double TaxaMudancaFD = (VelFD - VelAntigaFD)/DiferencaTempoAtual; //Calcula a taxa de mudança da velocidade dos NEOs.
    double TaxaMudancaFE = (VelFE - VelAntigaFE)/DiferencaTempoAtual; //
    double TaxaMudancaTE = (VelTE - VelAntigaTE)/DiferencaTempoAtual; //
    double TaxaMudancaTD = (VelTD - VelAntigaTD)/DiferencaTempoAtual; //
    
    if(Math.abs(TaxaMudancaFD)>TaxaMax){                                                 //Se a taxa de mudança for maior que a taxa máxima,
      VelFD = (VelAntigaFD + ((TaxaMax*DiferencaTempoAtual)*Math.signum(TaxaMudancaFD)));// troca a velocidade por uma que faz com que a 
    }                                                                                    // taxa nova seja menor que a máxima.
    if(Math.abs(TaxaMudancaFE)>TaxaMax){                                                 //
      VelFE = (VelAntigaFE + ((TaxaMax*DiferencaTempoAtual)*Math.signum(TaxaMudancaFE)));//
    }                                                                                    //
    if(Math.abs(TaxaMudancaTE)>TaxaMax){                                                 //
      VelTE = (VelAntigaTE + ((TaxaMax*DiferencaTempoAtual)*Math.signum(TaxaMudancaTE)));//
    }                                                                                    //
    if(Math.abs(TaxaMudancaTD)>TaxaMax){                                                 //
      VelTD = (VelAntigaTD + ((TaxaMax*DiferencaTempoAtual)*Math.signum(TaxaMudancaTD)));//
    }          

    VelAntigaFD = VelFD;  //Guarda a velocidade antiga pra possibilitar a conta acima
    VelAntigaFE = VelFE;  //  
    VelAntigaTE = VelTE;  //
    VelAntigaTD = VelTD;  //



    VelFD = VelFD * (Math.cos(Math.toRadians(Math.abs(EncoderFDCorrigido) - Math.abs(AnguloFD))));//Aplica a correção de cosseno
    VelFE = VelFE * (Math.cos(Math.toRadians(Math.abs(EncoderFECorrigido) - Math.abs(AnguloFE))));//
    VelTE = VelTE * (Math.cos(Math.toRadians(Math.abs(EncoderTECorrigido) - Math.abs(AnguloTE))));//
    VelTD = VelTD * (Math.cos(Math.toRadians(Math.abs(EncoderTDCorrigido) - Math.abs(AnguloTD))));//
   

    
    TracaoFD.set(VelFD);// Manda o valor calculado para o motor.
    TracaoFE.set(VelFE);//
    TracaoTE.set(VelTE);//
    TracaoTD.set(VelTD);//

  }
////////////////////////////////////////////////////////////////////////////////
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {

   DiferencaTempoAtual = Timer.getFPGATimestamp() - TempoAtual;
   TempoAtual = Timer.getFPGATimestamp();


   if(Math.abs(Controle_0.getRawAxis(0))<0.08 && Math.abs(Controle_0.getRawAxis(1))<0.08) {Joystick_X = 0;}// Faz a correção de zona morta do controle.
   else {Joystick_X = Controle_0.getRawAxis(0);}                                                                 // 
   //                                                                                                                 // Se o analógico esquerdo tiver os dois valores abaixo de 8%, eles são arredondados para 0. 
   //                                                                                                                 // Se o analógico direito tiver o seu valor abaixo de 8%, ele é arredondado para 0.
   if(Math.abs(Controle_0.getRawAxis(0))<0.08 && Math.abs(Controle_0.getRawAxis(1))<0.08) {Joystick_Y = 0;} //  
   else {Joystick_Y = Controle_0.getRawAxis(1);}                                                                 //  
   //                                                                                                                 //  
   //                                                                                                                 //  
   if(Math.abs(Controle_0.getRawAxis(4))<0.08) {Joystick_R = 0;}                                                 //  
   else {Joystick_R = Controle_0.getRawAxis(4);}                                                                 //  



    EncoderFDCorrigido = (NormalizarAngulo((EncoderFD.getAbsolutePosition()*349)+0));//TODO   Colocar o offset certo para o seu encoder no lugar do 0
    EncoderFECorrigido = (NormalizarAngulo((EncoderFE.getAbsolutePosition()*349)+0));//
    EncoderTECorrigido = (NormalizarAngulo((EncoderTE.getAbsolutePosition()*349)+0));//
    EncoderTDCorrigido = (NormalizarAngulo((EncoderTD.getAbsolutePosition()*349)+0));//


  }

  @Override
  public void autonomousInit() {
    UltimoAngulo_FD = EncoderFDCorrigido;
    UltimoAngulo_FE = EncoderFECorrigido;
    UltimoAngulo_TE = EncoderTECorrigido;
    UltimoAngulo_TD = EncoderTDCorrigido;
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    UltimoAngulo_FD = EncoderFDCorrigido;
    UltimoAngulo_FE = EncoderFECorrigido;
    UltimoAngulo_TE = EncoderTECorrigido;
    UltimoAngulo_TD = EncoderTDCorrigido;
  }

  @Override
  public void teleopPeriodic() {
    MovimentarSwerve(Joystick_X, Joystick_Y, Joystick_R);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
