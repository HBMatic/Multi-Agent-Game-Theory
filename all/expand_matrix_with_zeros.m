function new_matrix = expand_matrix_with_zeros(original_matrix, rows_to_add, cols_to_add)
    % Size of the original matrix
    [original_rows, original_cols] = size(original_matrix);
    
    % Calculate the size of the new matrix
    new_rows = original_rows + length(rows_to_add);
    new_cols = original_cols + length(cols_to_add);
    
    % Initialize the new matrix with zeros
    new_matrix = zeros(new_rows, new_cols);
    
    % Convert rows_to_add and cols_to_add to logical indices for simplicity
    row_mask = false(1, new_rows);
    row_mask(rows_to_add) = true;
    
    col_mask = false(1, new_cols);
    col_mask(cols_to_add) = true;
    
    % Variables to track the position in the original matrix
    original_row = 1;
    original_col = 1;
    
    for i = 1:new_rows
        if row_mask(i)
            continue; % Skip this row as it's one of the new rows to add
        end
        
        original_col = 1; % Reset original column index for each new row
        for j = 1:new_cols
            if col_mask(j)
                continue; % Skip this column as it's one of the new columns to add
            end
            new_matrix(i, j) = original_matrix(original_row, original_col);
            original_col = original_col + 1;
        end
        original_row = original_row + 1;
    end
end
