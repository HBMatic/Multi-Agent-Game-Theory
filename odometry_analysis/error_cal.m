name = input('Enter file name: ', 's');


output = {x, y, odomlog_gazebo, odomlog_tb3};

save(['gaz_vs_tb_ode/tb_3/', name, '.mat'], 'output');
fileName = ['documentation/tb_2/', name, '.txt'];
fileID = fopen(fileName, 'w');
if fileID == -1
    error('Failed to create file: %s', fileName);
end

maxRows = max(cellfun(@(data) size(data, 1), output));

for row = 1:maxRows
    for col = 1:length(output)
        data = output{col};
        [rows, cols] = size(data);
        
        if row <= rows
            fprintf(fileID, '%-10.6f', data(row, :));
        else
            fprintf(fileID, '%-10.6f', NaN(1, cols));
        end
    end
    fprintf(fileID, '\n');
end

fclose(fileID);

