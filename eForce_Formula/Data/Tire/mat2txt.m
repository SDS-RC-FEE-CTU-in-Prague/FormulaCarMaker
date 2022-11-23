clc;

%formatSpec = '%4.0f %3.0f %04.2f\r\n' % for longitudinal use
formatSpec = '%4.0f %3.0f %04.2f %03.2f\r\n' % for lateral use
fileID = fopen('xls2txt2.txt', 'w')

[nrows, ncols] = size(data);
for row = 1:nrows
    fprintf(fileID, formatSpec, data(row,:));
end

fclose(fileID);