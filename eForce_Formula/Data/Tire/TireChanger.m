clc;
clear;

%Data file name
name = 'FS_205_40R10_TYDEX.tdx';

FID = fopen(name, 'r');

%Get file pointer to MEASURCHANNELS
tline = 0;
while(~strcmp(tline, '**MEASURCHANNELS'))
    tline = fgets(FID);
    
    %Very stupid way of removing some type of character at end of line
    tline((length(tline)-1):length(tline)) = '';
end

%Get file pointer through comments to measurchannels names
tline = '!';
while(tline(1) == '!')
    tline = fgets(FID);
end

i=0;
while(length(tline) > 2) % Stop condition if line is too small
    i=i+1;
    
    i1=1;
    while(tline(i1) ~= ' ')
        i1=i1+1;
    end
    
    channel.name{i} = tline(1:i1-1); %pass channel names to
    
    tline = fgets(FID);
end
%i1 = i1-1; % correct the name count

%Get file pointer to MEASURDATA
tline = 0;
while(~strcmp(tline, '**MEASURDATA'))
    tline = fgets(FID);
    
    %Very stupid way of removing some type of character at end of line
    tline((length(tline)-1):length(tline)) = '';
end

%Get data
n_ch = i1+1; %Channel number correction
i_d =0; %Data pointer
while(1)
    
    tline = fgets(FID); % get line
    if(length(tline)<3)%stupid way to move when there is blank line
        tline = fgets(FID); % get line
        disp('jumped over blank line');
    else
        i_d = i_d+1; % Data pointer increase
    end
    
    %Very stupid way of removing some type of character at end of line
    tline((length(tline)-1):length(tline)) = '';
    len = length(tline);
    
    if(strcmp(tline, '**END')) % Ukoncovacia podmienka
        break;
    end
    
    i_start = 1;
    i_fin = 1;
    for i1 = 1:n_ch % while saving data
        
        while(tline(i_start) == ' ') % while reading white spacex
            i_start=i_start+1;
            if(i_start > len)
                i_start = len;
                break;
            end
        end 
        i_fin = i_start;
        
        while(tline(i_fin) ~= ' ') % while reading characters
            i_fin=i_fin+1;
            if(i_fin > len)
                i_fin = len;
                break;
            end
        end
        channel.data{i_d,i1} = str2num(tline(i_start:i_fin));
        i_start = i_fin;
    end
    
end




fclose(FID);