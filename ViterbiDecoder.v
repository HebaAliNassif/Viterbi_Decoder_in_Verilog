module ViterbiDecoder (
	clk,
	reset,
	restart,
	enable,
	encoded,
	decoded,
	error,
	ready,
	load,
	state_address,
	input_address,
	next_state_data,
	output_data
);

parameter n = 2;	// The convolutional encoder takes k-bits every clk cycle and emits n-bits
					// Output bits length

parameter k = 1;	// Input bits length

parameter m = 4;	// The generator size which is equal to k + the number of bits in the state register

parameter L = 7;	// The number of clock cycles before the state returns to zero. So per example, the decoder will take n*L bits and return k*L bits
					// Message length
					
// This is a function that calculated the ceil(log2(x)) for any integer x
function integer clog2;
input integer value;
begin
value = value-1;
for (clog2=0; value>0; clog2=clog2+1) value = value>>1;
end
endfunction

// The maximum number of errors in an example is L*n so we should represent the error in clog2(L*n)+1 bits
localparam E = clog2(L*n);

//The count of states
localparam states_count = 2**(m-k);

input  clk;							// The clock signal.
input  reset; 						// The asynchronous reset signal (reset the whole decoder).
input  restart;						// This signal tells the decoder to discard its result and start processing a new stream of encoded bits.
input  enable;						// This signal can be used to pause the processing if equal zero.
input  [0:n-1] encoded; 			// The encoded bits for this clock cycle
output [0:k*L-1] decoded; 			// The result of the decoding process.
output [0:E] error; 				// The number of errors found in the most likely sequence
output ready; 						// Whether the decoder finished processing and the output is ready
input  load; 						// This signal tells the decoder to read an entry to the Next State Table and the Output table
input  [0:m-k-1] state_address;		// The address of the state in the tables into which we want to write (the row according to the tables in the project description)
input  [0:k-1] input_address; 		// The address of the input in the tables into which we want to write (the column according to the tables in the project description)
input  [0:m-k-1] next_state_data; 	// The next state to write into the Next State Table
input  [0:n-1] output_data; 		// The output to write into the Output Table

//variable declarations
integer i;
integer j;
integer q;
integer count;
integer t_table = 0;												

reg [0:2**(m-k)-1] reachable_state;
reg [0:k-1] input_was_at_the_reachable_state;
reg [0:n-1] output_was_at_the_reachable_state;
reg [0:k*L-1] decoded_reg; 

reg [0:(m-k)-1] state_table [(2**(m-k))-1:0][(2**k)-1:0]; 			// rows (current state): 2**(m-k) possible value for the states
																	// columns (if input is): 2**k possible value for the input
																	// cell (next state): length of each cell is (m-k)as it represents a state number 
					
reg [0:n-1] output_table [(2**(m-k))-1:0][(2**k)-1:0]; 				// rows (current state): 2**(m-k) possible value for the states
																	// columns (if input is): 2**k possible value for the input
																	// cell (output): length of each cell is n output bits
	
reg [0:k-1] convolutional_table [(2**(m-k))-1:0][(2**(m-k))-1:0];	// rows (current state): 2**(m-k) possible value for the states
																	// columns (next state):  2**(m-k) possible value for the states																// cell (input was): length of each cell is k as it represents the input bits that can make a change to a state 

reg [0:(m-k)-1] history_table [(2**(m-k))-1:0][L:0];				// rows (current state): 2**(m-k) possible value for the states
																	// columns (message current position): L is the length of the message
																	// cell (previous state): length of each cell is (m-k)as it represents a state number
																	
reg [0:E] error_table [(2**(m-k))-1:0][L:0];						// rows (current state): 2**(m-k) possible value for the states
																	// columns (message current position): L is the length of the message
																	// cell (cumulative state error): length of each cell is E as it's an error value

reg visited [(2**(m-k))-1:0][L:0];									// rows (current state): 2**(m-k) possible value for the states
																	// columns (message current position): L is the length of the message
																	// cell (visited state): a bit that represents if the state was visited from a state from the previous column

reg [0:k-1] input_was_to_reach_state [(2**(m-k))-1:0][L:0];			// rows (current state): 2**(m-k) possible value for the states
																	// columns (message current position): L is the length of the message
																	// cell (input was): contains the input thas was used to reched the state

reg ready_reg = 1'b0;

reg [(2**(m-k))-1:0] minimum_error_state;


always @(posedge clk ) begin
	//Loading data in tables
	if(load == 1) begin
		
		state_table[state_address][input_address] = next_state_data;
		output_table [state_address][input_address] = output_data;
		convolutional_table [state_address][next_state_data] = input_address;
		
	end
	
	else if(reset == 1) begin
		visited [0][0] = 1;
		error_table[0][0] = 0;
		history_table[0][0] = 0;
	end
	
	//On restart, reset tables.
	else if(restart == 1) begin
		for (i = 0; i < (2**(m-k)); i = i+1) begin
			for (j = 0;j <= L; j = j+1) begin
				error_table[i][j] = 0;
				history_table[i][j] = 0;
				visited [i][j] = 0;
				
			end
		end
		visited [0][0]=1;
		t_table = 0;
		ready_reg = 0;		
	end
	
	// Make necessary modifications for the next example.
	else if(enable==1) begin
		if(ready_reg==0)begin
			for(i = 0; i < (2**(m-k)); i = i+1 )begin						//Loop through every possible state
				if(visited[i][t_table]==1)begin								//If the state was visited before then it needs calculation
					for(j = 0; j < (2**k); j = j+1 )begin					//Loop through every possible state that can be reached
						reachable_state = state_table[i][j];
						output_was_at_the_reachable_state = output_table[i][j];
						input_was_at_the_reachable_state = convolutional_table[i][reachable_state];
						count = 0;
						for(q = 0; q < n; q = q+1)begin										//Calculate the hamming distance between the encoded and the expected output
							if((output_was_at_the_reachable_state[q]^encoded[q]) == 1)begin
								count = count + 1;
							end
						end
						if(visited[reachable_state][t_table+1] == 1)begin										//If the state was already visited before
							if(error_table[reachable_state][t_table+1]> count + error_table[i][t_table])begin	//Compare the errors and update the tables if the new calculations have less error
								history_table[reachable_state][t_table+1] = i;
								error_table[reachable_state][t_table+1] = count + error_table[i][t_table];
								visited[reachable_state][t_table+1] = 1;
								input_was_to_reach_state[reachable_state][t_table+1] = input_was_at_the_reachable_state;
							end
						end
						else begin															//If the state was not visited before then fill the tables
							history_table[reachable_state][t_table+1] = i;
							error_table[reachable_state][t_table+1] = count + error_table[i][t_table];
							visited[reachable_state][t_table+1] = 1;
							input_was_to_reach_state[reachable_state][t_table+1] = input_was_at_the_reachable_state;
						end
					end
				end
			end
			t_table = t_table + 1;											//Next column
		end
		if(t_table==L) begin

			minimum_error_state = 0;
			for(i = 1; i < (2**(m-k)); i = i+1 )begin 						//Loop through the last column cells in the error table to get the minimum error
				if(error_table[minimum_error_state][t_table] > error_table[i][t_table])begin
					minimum_error_state = i;
				end
			end
			
			for(i = L-1; i >= 0; i = i-1 )begin								//Loop through columns of the tables to do backtracking
				decoded_reg[k*i+:k] = input_was_to_reach_state[minimum_error_state][i+1];
				minimum_error_state = history_table[minimum_error_state][i+1];
			end
			
			ready_reg = 1;
		end
	end
end
assign ready = ready_reg;
assign decoded = decoded_reg;
//// The following blocks are for debugging purposes.
//// They print the content of the created tables.
//always @(negedge load) begin
//	$display("State Table:");
//	for(i=0; i<(2**(m-k)); i=i+1) begin
//		for(j=0; j<(2**k); j=j+1) begin
//			$write("state_table[%0d][%0d] = %0d\t", i, j, state_table[i][j]);
//		end
//		$write("\n");
//	end
//	
//	$display("Output Table:");
//	for(i=0; i<(2**(m-k)); i=i+1) begin
//		for(j=0; j<(2**k); j=j+1) begin
//			$write("output_table[%0d][%0d] = %0b\t", i, j, output_table[i][j]);
//		end
//		$write("\n");
//	end
//
//	$display("Convolutional Table:");
//	for(i=0; i<(2**(m-k)); i=i+1) begin
//		for(j=0; j<(2**(m-k)); j=j+1) begin
//			$write("output_table[%0d][%0d] = %0d\t", i, j, convolutional_table[i][j]);
//		end
//		$write("\n");
//	end
//end
//
//always @(negedge enable) begin
//	$display("History Table:");
//	for(i = 0; i < (2**(m-k)); i = i+1) begin
//		for(j = 0; j <= L; j = j+1) begin
//			$write("history_table[%0d][%0d] = %0d\t", i, j, history_table[i][j]);
//		end
//		$write("\n");
//	end
//	
//	$display("Error Table:");
//	for(i = 0; i < (2**(m-k)); i = i+1) begin
//		for(j = 0; j <= L; j = j+1) begin
//			$write("error_table[%0d][%0d] = %0d\t", i, j, error_table[i][j]);
//		end
//		$write("\n");
//	end
//	
//	$display("Input to reach Table:");
//	for(i = 0; i < (2**(m-k)); i = i+1) begin
//		for(j = 0; j <= L; j = j+1) begin
//			$write("input_to_reach[%0d][%0d] = %0b\t", i, j, input_was_to_reach_state[i][j]);
//		end
//		$write("\n");
//	end
//	
//	$display("Visited Table:");
//	for(i = 0; i < (2**(m-k)); i = i+1) begin
//		for(j = 0; j <= L; j = j+1) begin
//			$write("visited[%0d][%0d] = %0b\t", i, j, visited[i][j]);
//		end
//		$write("\n");
//	end
//end

endmodule