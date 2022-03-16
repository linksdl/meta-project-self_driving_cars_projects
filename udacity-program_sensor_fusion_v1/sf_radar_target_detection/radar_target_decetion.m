clear all
clc;

% 雷达具体参数

frequency = 77e9;    % Operating carrier frequency of Radar 
range_resolution = 1; % range resolution can determine bandwith
max_range = 200;
max_velocity = 70;    % max velocity can determine Ts_x-axis
velocity = 3; 

% 目标的初始距离和速度
target_start_position = 100; % Range cannot exceed the max value of 200m
target_start_velocity = 50;  % velocity can be any value in the range of -70 to + 70 m/s.


% 1, FMCW 设置
c = 3e8; % 光速
wave_lenght = c/frequency;
band_width = c/(2*range_resolution); % The sweep bandwidth can be determined according to the range resolution and the sweep slope is calculated using both sweep bandwidth and sweep time.
Tchirp = 5.5 * 2 * max_range / c; % Tchirp=5.5⋅2⋅Rmax/c
slope = band_width/Tchirp; % Giving the slope of the chirp signal  Slope=Bandwidth/Tchirp
% 2, 移动目标生成
% for 128 chrips, each chrips send 1024 times.
Nd = 128;
Nr = 1024;

t = linspace(0, Nd*Tchirp, Nr*Nd); % total time for samples
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));

fc = frequency;
% 3，信号处理
target_position = target_start_position;
for i = 1:length(t)

target_position = target_start_position + target_start_velocity*t(i);

time_delay = (2*target_position)/c; %compute time_delay.

%For each time sample we need update the transmitted and received signal. 
Tx(i) = cos(2*pi*(fc*t(i)+slope*t(i)*t(i)/2));
Rx(i) = cos(2*pi*(fc*(t(i)-time_delay)+slope*(t(i)-time_delay)*(t(i)-time_delay)/2));

% Now by mixing the Transmit and Receive generate the beat signal
% This is done by element wise matrix multiplication of Transmit and
% Receiver Signal
% Mixed or Beat Signal = (Tx.*Rx)
Mix(i) = Tx(i)*Rx(i);

end

% 4，Range/Doppler FFT
% reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
% Range and Doppler FFT respectively.
Mix = reshape(Mix,[Nr,Nd]);

% run the FFT on the beat signal along the range bins dimension (Nr) and normalize.
Y=fft(Mix,Nr,1);

% Take the absolute value of FFT output
P2=abs(Y/Nr);

% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
P1=P2(1:Nr/2);


P1=fftshift(P1); %important

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

% *%TODO* :
% plot FFT output 
range = linspace(-200,200,Nr/2)*((Nr/2)/400);

plot(range, P1);
axis ([0 200 0 1]);


% 5, FFT 2D

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);


% 7, CFAR detection
% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr=10;
Td=8;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr=4;
Gd=4;

% *%TODO* :
% offset the threshold by SNR value in dB
offset=6;

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
RESULT=zeros(Nr/2,Nd);

for i= Tr+Gr+1: Nr/2-(Gr+Tr)
    for j= Td+Gd+1:Nd-(Gd+Td)
        noise_level=0;
        train_cell=0;
        %sliding the grid
        for p= i-(Tr+Gr):i+Tr+Gr
            for q= j-(Td+Gd):j+Td+Gd
                if(abs(i-p)>Gr||abs(j-q)>Gd)
                    noise_level=noise_level+db2pow(RDM(p,q));
                    train_cell=train_cell+1;
                end
            end
        end
        
        noise_average=noise_level/train_cell;
        noise_threshold=pow2db(noise_average);
        noise_threshold=noise_threshold+offset;
        
        CUT=RDM(i,j);
        if(CUT<noise_threshold)
            RESULT(i,j)=0;
        else
            RESULT(i,j)=1;
        end
    end
end

%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,RESULT);
colorbar;
