function sistemadecontrolePI
clear all;
delete(timerfindall);
%
% parametros do controlador PI H(s) = Kp(1 + 1/Tis)
%
Kp = 1;
Ti = 0.5;
% parametros do tempo
Fa = 10;         % Sampling frequency
T = 1/Fa;        % Sampling time
Duration = 50;   % Duracao em segundos
%%%%%
NumberOfTasksToExecute = round(Duration/T);  % <---- Numero de vezes que o controlador e' executado
%%%%%
% Criacao de uma onda quadrada simetrica de referencia com periodo Per
% Amplitude +A, -A
A = 3.0;
Per = 15;
referencia = zeros(NumberOfTasksToExecute+1,1);
t = linspace(0,Duration,NumberOfTasksToExecute+1);
referencia = A*square(2*pi*(1/Per)*t);

% Create and configure timer object
tm = timer('ExecutionMode','fixedRate', ...            % Run continuously
    'Period',T, ...                                    % Period = sampling time
    'TasksToExecute',NumberOfTasksToExecute, ...       % Runs NumberOfTasksToExecute times
    'TimerFcn',@MyTimerFcn, ...                        % Run MyTimerFcn at each timer event
    'StopFcn',@StopEverything);
% setup da placa
s = daq.createSession('ni');
addAnalogInputChannel(s,'Dev1',0,'Voltage');
addAnalogOutputChannel(s,'Dev1',0,'Voltage');
s.Rate = Fa;
%
% Como setar os parametros
%
% Parametros do controlador PID
%         b1*z^2 + b2*z + b3
% H(z) = --------------------
%         a1*z^2 + a2*z + a3
%
% Controlador PI
%         b1*z + b2
% H(z) = -----------
%         a1*z + a2
% fazer b3=0 a3=0
%
% Coeficientes para PI com transformacao bilinear
a1 =  2*Ti;
a2 = -2*Ti;
a3 = 0.0;
b1 = T*Kp+2*Ti*Kp;
b2 = T*Kp-2*Ti*Kp;
b3 =  0.0;
%a1 =  1;
%a2 = 0.0;
%a3 = 0.0;
%b1 = 2;
%b2 = 0.0;
%b3 =  0.0;
% Controlador P
%
% H(z) = b1 = Kp
% fazer a1=1, a2=0, a3=0, b2=0, b3=0
% 
% a1 =  1.0;
% a2 =  0.0;
% a3 =  0.0;
% b1 =  1.0; % Kp
% b2 =  0.0;
% b3 =  0.0;

% inicializacao de variaveis do sistema de controle
rk   = 0.0;
yk   = 0.0;
%
uk   = 0.0;
uk_1 = 0.0;
uk_2 = 0.0;
%
ek   = 0.0;
ek_1 = 0.0;
ek_2 = 0.0;
%
% Inicializacao do vetor que guarda o historico das variaveis
vetorrk = zeros(NumberOfTasksToExecute+1,1);
vetorrk(1) = 0; % instante 0 -> k=1
vetorrk(2) = 0;
vetoryk = zeros(NumberOfTasksToExecute+1,1);
vetoryk(1) = 0;
vetoryk(2) = 0;
vetoruk = zeros(NumberOfTasksToExecute+1,1);
vetoruk(1) = 0;
vetoruk(2) = 0;
% inicializacao da variavel do tempo discreto
k=3;
% Start the timer
start(tm);
%
% Funcao que e' executada no timer tm periodicamente a cada T    <--------------------------------------
%
function MyTimerFcn(~,~)    
% Leitura da Porta A/D
sample = inputSingleScan(s);     % le o dado do canal de entrada 0    
rk = referencia(k);
% Escolher aqui ou velocidade angular ou posicao angular
yk = (-1)*sample(1);   % leitura da saida do amp op - troca o sinal
                       % devido a configuracao inversora
%
% calculo do erro e(k)
%
ek = rk - yk;    
% Calculo do controle
uk = (-a2/a1)*uk_1+(-a3/a1)*uk_2+(b1/a1)*ek+(b2/a1)*ek_1+(b3/a1)*ek_2;
% Controle de Saturacao
if uk >= 10.0 
   uk = 10.0;
else if uk <= -10.0
   uk = -10.0;
    end
end % fim da saturacao
% Escrita na Porta D/A - u(k)
outputSingleScan(s,uk);
% Salva os valores
vetorrk(k) = rk;
vetoryk(k) = yk;
vetoruk(k) = uk;
% Update de variaveis    
k = k+1;
uk_2 = uk_1; 
uk_1 = uk;
ek_2 = ek_1;   
ek_1 = ek;
end % Fim da funcao MyTymerFcn
% Funcao executada ao final da execucao do timer tm
function StopEverything(~,~)
    outputSingleScan(s,0.0);
    release(s);
    nsamples = length(vetorrk);
    t=0.0:T:(nsamples-1)*T;
    plot(t,vetorrk,'-',t,vetoryk,'--');
    hold on
    stairs(t,vetoruk);
    grid on
    title('Referencia r(k) / Esforco de controle u(k) /    Saida da planta y(k)')
    xlabel('tempo (s)');
    ylabel('tensao (Volts)');
    mensagem='acabou'
end % Fim de StopEverything

end  % FIM function controlador
