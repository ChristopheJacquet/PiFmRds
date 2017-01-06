

% Filter analysis for combined LPF and preemphasis 
% IIR reference material:  http://jontio.zapto.org/hda1/preempiir.pdf
%
% Tested on GNU Octave, should work on Matlab

cutoff_freq=15700
if 1
  FIR_PHASES=32      % Values for PiFmRds
  sample_rate=44100  
  tau=75e-6          
  delta=1.96e-6      
else
  FIR_PHASES=4       % Values used in reference material
  sample_rate=48000  % 48000*4 = 192000
  tau=50e-6          % to verify results against example values
  delta=7.96e-6      
end


FIR_SIZE=1024

taup=1/(2*sample_rate*FIR_PHASES)*cot(1/(2*tau*sample_rate*FIR_PHASES))
deltap=1/(2*sample_rate*FIR_PHASES)*cot(1/(2*delta*sample_rate*FIR_PHASES))
bp=sqrt(-(taup^2) + sqrt( (taup.^4) + 8*(taup^2)*(deltap^2) )  )/2
ap=sqrt(2*(bp^2) + (taup^2) )
a0=( 2*ap + 1/(sample_rate*FIR_PHASES) )/(2*bp + 1/(sample_rate*FIR_PHASES) )
a1=(-2*ap + 1/(sample_rate*FIR_PHASES) )/(2*bp + 1/(sample_rate*FIR_PHASES) )
b1=( 2*bp - 1/(sample_rate*FIR_PHASES) )/(2*bp + 1/(sample_rate*FIR_PHASES) )

x=0;
y=0;

for j=1:FIR_SIZE
  sincpos=j-((FIR_SIZE+1)/2);
  if (sincpos==0)
    firlowpass(j)=  2 * cutoff_freq / (sample_rate*FIR_PHASES) ;
    disp('Oops: sincpos hit zero');
  else
    firlowpass(j)= sin(2 * pi * cutoff_freq * sincpos / (sample_rate*FIR_PHASES) ) / (pi * sincpos) ;
  end
  y=a0*firlowpass(j) + a1*x + b1*y;
  x=firlowpass(j);
  firpreemph(j)=y;
  win(j)=(.54 - .46 * cos(2*pi * (j) / (FIR_SIZE))) * FIR_PHASES ; 

end

fullfilt=firpreemph.*win;

hold off

for j=1:FIR_PHASES
  phasefreqs=linspace(0,sample_rate*FIR_PHASES*((FIR_SIZE/FIR_PHASES)-1)/FIR_SIZE,FIR_SIZE/FIR_PHASES);
  phasedb=20*log10(abs(fft(fullfilt(j:FIR_PHASES:FIR_SIZE))));
  % real data, so only show the positive frequencies
  l1=plot( phasefreqs(1:(end/2)+1) , phasedb(1:(end/2+1)));
  hold on

  %plot( fullfilt(j:FIR_PHASES:FIR_SIZE) )
end

fulldb= 20*log10(abs(fft(fullfilt))/FIR_PHASES);
fullfreqs=linspace(0,sample_rate*FIR_PHASES*(FIR_SIZE-1)/FIR_SIZE,FIR_SIZE);
l2=plot( fullfreqs(1:(end/2+1)) ,fulldb(1:(end/2+1)),'r' );


l3=plot( [19000 19000 ] , [0 -59] , 'g' ,'linewidth',2 ); % Pilot frequency 


legend([l1,l2,l3],'Individual Phases','Full Filter Response','Pilot Tone')

title('PiFmRds Polyphase Filter Response')
xlim([0 60000]);
xlabel('Hz');
ylabel('dB');
hold off

% semilogx( linspace(0,sample_rate*FIR_PHASES*(FIR_SIZE-1)/FIR_SIZE,FIR_SIZE) ,20*log10(abs(fft(fullfilt))/FIR_PHASES) , 'r' )

