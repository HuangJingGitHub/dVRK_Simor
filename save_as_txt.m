clear
load( "dvrk_mtm_psm.mat");

fileID = fopen('mtm_q.txt', 'wt');
[row, col] = size(mtm_q);
for i = 1: 1 : col
    for j = 1: 1: row
        if j == row
            fprintf(fileID, '% 6.4f \n', mtm_q(j, i));
        else
            fprintf(fileID, '% 6.4f \t', mtm_q(j, i));
        end
    end
end
%fprintf(fileID, '%6.4f\n', mtm_q);
fclose(fileID);