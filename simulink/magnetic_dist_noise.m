function z = magnetic_dist_noise(n)
% o = magnetic_dist_noise(n)

% perfect signal generation
m = round(n/10);            % block size
s = (randperm(5)-1)*2+1;    % start location
a = randn(1,5);             % amplitudes
freq = 0.5+2*rand(2,1);     % frequencies for sin
x = zeros(n,1);
% 1. square
x(s(1)*m:(s(1)+1)*m) = a(1);
% 2. sawtooth
x(s(2)*m:(s(2)+1)*m) = a(2) * (0:1/m:1);
% 3. sin freq1
x(s(3)*m:(s(3)+1)*m) = a(3) * sin(freq(1)*2*pi*(0:1/m:1));
% 4. sin freq2
x(s(4)*m:(s(4)+1)*m) = a(4) * sin(freq(2)*2*pi*(0:1/m:1));
% 5. triangle
x(s(5)*m:(s(5)+1)*m) = a(5) * [0:1/m:0.5 0.5-1/m:-1/m:0];

% hilbert
y = hilbert(x);
y = imag(y) + flipud(abs(y));

% filters
z = detrend(y,'linear');            % remove linear trend
[num, den]= butter(1,0.008,'low');   % lowpass filter
z = filter(num,den,z);
% output
o = [z x];

if 0
figure
plot([0:n-1]/1000,x)
hold on
plot([0:n-1]/1000,y)
hold on
plot([0:n-1]/1000,z)
close all
end
