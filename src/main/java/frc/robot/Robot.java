package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;


import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

public class Robot extends TimedRobot {
  CANSparkMax EsquerdaFrente;
  CANSparkMax DireitaFrente;
  CANSparkMax EsquerdaTraseira;
  CANSparkMax DireitaTraseira;

  MotorControllerGroup MotorControllerGroupDireito;
  MotorControllerGroup MotorControllerGroupEsquerdo;

  DifferentialDrive Drivetrain;

  final String Brake = "Brake";
  final String Coast = "Coast";

  CANSparkMax SparkExtensao;
  CANSparkMax SparkInclinacao;
  CANSparkMax SparkGarra;
  static boolean SegurancaAbertura;
  
  static RelativeEncoder EncoderChassi;
  Pigeon2 Pidgeotto;
  
  
  Joystick Controle_0;
  public final int Analog_Y = 1;
  public final int Analog_X = 4;
  static boolean auxiliar_5;
  
  static double TempoAtual;
  static double TempoCiclo;
  
  public final double TempoMaxAceleracao = 0.7;
  static boolean TimerLigado;
  static double T0;
  static double TempoAtualAceleracao;
  static double VelocidadeMin;
  static double DirecaoAnterior_Analog_Y;

  static double ErroGiro;

  static double Derivativa;
  
  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  public void AtualizarGarra(){ //TELEOP
    if (Controle_0.getRawButton(5) == true){
      SparkGarra.set(1);
    }
    else if (Controle_0.getRawButton(6) == true){
      SparkGarra.set(-0.5);
    }
    else{
      SparkGarra.set(0);
    }
  }

  public double AceleracaoTeleop(double VelocidadeMaxima){ //TELEOP
    double Controle_0_Analog_Y = -Controle_0.getRawAxis(Analog_Y);

    // Correção Velocidade Maxima: 
    VelocidadeMaxima -= 0.3;
    
    // Tempo aceleração:
    if (Math.abs(Controle_0_Analog_Y) >= 0.06 && DirecaoAnterior_Analog_Y == Math.signum(Controle_0_Analog_Y)){ 
      VelocidadeMin = 0.3;
      
      // Inicia o timer:
      if (TimerLigado == false){
        T0 = TempoAtual;
        TimerLigado = true;
      }
      
      // Atualiza o timer:
      TempoAtualAceleracao = TempoAtual - T0;
    }
    
    // Desliga o timer se EixoY = 0 ou se o sinal do EixoY mudar:
    else{
      TimerLigado = false;
      TempoAtualAceleracao = 0;
      VelocidadeMin = 0;
    }
    DirecaoAnterior_Analog_Y = Math.signum(Controle_0_Analog_Y);
    

  // Variáveis da operação da Velocidade de locomoção: 
  double Sentido_VelocidadeMin = VelocidadeMin * Controle_0_Analog_Y/Math.abs(Controle_0_Analog_Y);
  double LimiteAceleracao = Math.min((TempoAtualAceleracao / TempoMaxAceleracao),1);

  
  // Velocidade de locomoção:

  return (Controle_0_Analog_Y * (VelocidadeMaxima * LimiteAceleracao) + Sentido_VelocidadeMin);
}

  public void AtualizarMovimentoBraco(){ //TELEOP
    if(Controle_0.getRawButton(3) == true){
      SparkExtensao.set(0.25);
    }
    else if(Controle_0.getRawButton(2)){ 
      SparkExtensao.set(-0.25);
    }
    else if(Controle_0.getRawButton(3) == false && Controle_0.getRawButton(2) == false){
      SparkExtensao.set(0);

    }
    
    SparkInclinacao.set(((Controle_0.getRawAxis(2)-Controle_0.getRawAxis(3))*-0.6));
  }

  public double Aceleracao(double Distancia, double VelocidadeMaxima){ 
    
    double Distancia_Rampa = 40;
  
    double FatorConversao = 0.22;

    double Distancia_Rampa_Pulso = Distancia_Rampa * FatorConversao;
    double Distancia_Pulso = Distancia * FatorConversao;
    double Distancia_Constante = (Distancia - Distancia_Rampa) * FatorConversao;

    double VelocidadeMin = 0.2;

  
      if(EncoderChassi.getPosition() < Distancia_Constante){
        return VelocidadeMaxima;
         }
      else if ((EncoderChassi.getPosition() > Distancia_Constante) && (EncoderChassi.getPosition() < Distancia_Pulso)){
     
        double Velocidadeatual = ((VelocidadeMin - VelocidadeMaxima) / Distancia_Rampa_Pulso ) * (EncoderChassi.getPosition() - Distancia_Constante) + VelocidadeMaxima;
    
        return Velocidadeatual;
      }
      else
        return 0;
 
  }

  public double PGiroscopio(double Alvo){

      //PROPORCIONAL
      double UltimoErro = ErroGiro;
  
      double Kp = 0.06;
      ErroGiro = (Alvo - Pidgeotto.getYaw());
      double Proporcional = (ErroGiro * Kp);

      //DERIVATIVA
      double Kd = 0.40;
      Derivativa = (ErroGiro - UltimoErro) * Kd;

      //CORREÇÃO
      double Correcao = Math.max(Math.min(Proporcional + Derivativa, 0.55), -0.55);

      return Correcao;

  }

  public void BrakeCoastChassi(String ModoParada){
    if(ModoParada.charAt(0) == 'C'){
      EsquerdaFrente.setIdleMode(IdleMode.kCoast);
      EsquerdaTraseira.setIdleMode(IdleMode.kCoast);
      DireitaFrente.setIdleMode(IdleMode.kCoast);
      DireitaTraseira.setIdleMode(IdleMode.kCoast);
    }
    if(ModoParada.charAt(0) == 'B'){
      EsquerdaFrente.setIdleMode(IdleMode.kBrake);
      EsquerdaTraseira.setIdleMode(IdleMode.kBrake);
      DireitaFrente.setIdleMode(IdleMode.kBrake);
      DireitaTraseira.setIdleMode(IdleMode.kBrake);
    }
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

  @Override
  public void robotInit(){
  //SPARKS CHASSI
  DireitaFrente = new CANSparkMax(7,MotorType.kBrushless);
  DireitaTraseira = new CANSparkMax(11,MotorType.kBrushless);
  EsquerdaFrente = new CANSparkMax(6,MotorType.kBrushless);
  EsquerdaTraseira = new CANSparkMax(10,MotorType.kBrushless);

  EsquerdaFrente.restoreFactoryDefaults();
  EsquerdaTraseira.restoreFactoryDefaults();
  DireitaFrente.restoreFactoryDefaults();
  DireitaTraseira.restoreFactoryDefaults();
  
  MotorControllerGroupEsquerdo = new MotorControllerGroup(EsquerdaFrente, EsquerdaTraseira);
  MotorControllerGroupDireito = new MotorControllerGroup(DireitaFrente, DireitaTraseira);
  MotorControllerGroupDireito.setInverted(true);

  Drivetrain = new DifferentialDrive(MotorControllerGroupEsquerdo, MotorControllerGroupDireito);

  //SPARKS BRAÇO+GARRA
  SparkExtensao = new CANSparkMax(5, MotorType.kBrushed);
  SparkGarra = new CANSparkMax(9, MotorType.kBrushed);
  SparkInclinacao = new CANSparkMax(12, MotorType.kBrushed);

  SparkExtensao.setIdleMode(IdleMode.kBrake);
  SparkInclinacao.setIdleMode(IdleMode.kBrake);
  SparkGarra.setIdleMode(IdleMode.kBrake);
  
  //PIGEON
  Pidgeotto = new Pigeon2(17);
  Pidgeotto.configMountPose(AxisDirection.PositiveZ, AxisDirection.PositiveX);
  
  //CONTROLE(s)
  Controle_0 = new Joystick(0);

  EncoderChassi = EsquerdaFrente.getEncoder();
  }  

  @Override
  public void disabledInit(){
    BrakeCoastChassi(Coast);
  }

  @Override
  public void autonomousInit(){
  EncoderChassi.setPosition(0);

  BrakeCoastChassi(Coast);

  Pidgeotto.setYaw(0);
  }

  @Override
  public void teleopInit(){
    EncoderChassi.setPosition(0);
    BrakeCoastChassi(Coast);
    Pidgeotto.setYaw(0);
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void robotPeriodic(){ //atualiza timer do robô.
  TempoCiclo = Timer.getFPGATimestamp() - TempoAtual;
  TempoAtual = Timer.getFPGATimestamp();  


  System.out.println(Pidgeotto.getYaw());
  }  

  @Override
  public void autonomousPeriodic(){
   if(Pidgeotto.getYaw()<180){
    BrakeCoastChassi(Brake);
    Drivetrain.arcadeDrive(0,0.2);
  }
  // Drivetrain.arcadeDrive(0,PGiroscopio(0));
}

  @Override
  public void teleopPeriodic(){ //movimenta todos os eixos do chassi e garra manual
     Drivetrain.arcadeDrive(AceleracaoTeleop(0.55), Controle_0.getRawAxis(Analog_X)*-0.43);
    AtualizarGarra();
    AtualizarMovimentoBraco();

  }
 
  @Override
  public void disabledPeriodic(){}

























  @Override
  public void testInit(){}

  @Override
  public void testPeriodic(){}
}