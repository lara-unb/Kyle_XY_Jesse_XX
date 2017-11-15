function [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable(varname)
% gdatalogger_ipc_retrievevariable(varname) acessa a fila de dados do
% gdatalogger retornando o que ainda não foi lido da variável 'varname'. O
% datalogger está supostamente rodando em um outro processo do sistema
% operacional.
%
% Forma de chamada:
% [data,dataunitstring,status] = gdatalogger_ipc_retrievevariable(varname)
% 
% em que:
%     varname: string com o nome da variável, conforme definido no datalogger
%
%     data: vetor com os dados ainda não lidos da fila da variável 'varname'.
%     Se não houver novos dados, então retorna um vetor vazio.
%
%     dataunitstring: string da unidade em que os valores estão representados.
%
%     status: indica o tipo de resposta, podendo ser:
%         2: há dados disponíveis, copiados em data
%         3: a variável com nome "varname' não existe
%         4: estouro de tempo na solicitação. Provavelmente o datalogger
%            não está mais rodando

