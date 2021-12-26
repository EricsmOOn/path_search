%%----------------------------------------------------
%% @doc
%% Grid A* Search
%% 此A*寻路算法只适用于二维网格型地图
%% 使用前需配置网格长(define LENGTH)宽(define LENGTH)
%% 使用前需配置所有可行走网格(fun walkable/1)
%% @author Eric Wong
%% @end
%% Created : 2021-12-25 16:59 Saturday
%%----------------------------------------------------
-module(a_star).
-export([search/2, search/3]).

-define(ABS(__FLOAT__), erlang:abs(__FLOAT__)).
-define(MIN(__N1__, __N2__), erlang:min(__N1__, __N2__)).
-define(MAX(__N1__, __N2__), erlang:max(__N1__, __N2__)).
-define(HYPOTENUSE(__N1__, __N2__), math:sqrt(__N1__ * __N1__ + __N2__ * __N2__)).

-define(WIDTH, 1).
-define(LENGTH, 1).
-define(HYPOTENUSE, ?HYPOTENUSE(?WIDTH, ?LENGTH)).

-define(D(__N__), io:format("DEBUG[LINE:~w] ~w~n", [?LINE, __N__])).

-record(node, {
          %% 节点位置
          pos
          %% 节点前置位置
          ,parent
          %% 节点代价 {起点代价, 终点预计代价}
          ,cost
         }).

%%----------------------------------------------------
%% 配置接口
%%----------------------------------------------------
%% @doc 配置地图可行走区域 输入坐标 返回true|false
-spec walkable(Pos::{integer, integer}) -> boolean().
walkable({0, 0}) -> true;
walkable({0, 1}) -> true;
walkable({0, 2}) -> true;
walkable({1, 2}) -> true;
walkable({2, 0}) -> true;
walkable({2, 1}) -> true;
walkable({2, 2}) -> true;
walkable(_) -> false.

%%----------------------------------------------------
%% 外部接口
%%----------------------------------------------------
%% @doc 搜索入口
-spec search({pos_integer, pos_integer}, {pos_integer, pos_integer}) -> [{pos_integer, pos_integer}] | {false, Reason::bitstring()}.
search(Start = {_, _}, Goal = {_, _}) ->
    search(Start, Goal, fun walkable/1).

search(Start = {_, _}, Goal = {_, _}, WalkableFun) when is_function(WalkableFun, 1) ->
    case WalkableFun(Start) andalso WalkableFun(Goal) of
        true -> 
            case do([#node{pos = Start, cost = calc_score(Start, Start, {0, 0}, Goal)}], [], Goal, WalkableFun) of
                FinalSet = [_ | _] ->
                    trace_back(FinalSet, Start);
                Err = {false, _} ->
                    Err
            end;
        false ->
            {false, <<"UNREACHABLE AREA">>}
    end.

%%----------------------------------------------------
%% 内部私有
%%----------------------------------------------------
%% 递归主流程
do([], _, _, _) -> {false, <<"UNREACHABLE AREA">>};
do(OpenSet = [#node{pos = {X, Y}} | _], CloseSet, {X, Y}, _) ->
    OpenSet ++ CloseSet;
do(OpenSet = [Node = #node{pos = Pos = {_, _}, cost = Cost} | T], CloseSet, Goal, WalkableFun) ->
    AllSet = CloseSet ++ OpenSet,
    AddNodes = [#node{pos = NearPos, parent = Pos, cost = calc_score(NearPos, Pos, Cost, Goal)} || NearPos <- near_nodes(Pos), not lists:keymember(NearPos, #node.pos, AllSet), WalkableFun(NearPos)],
    NOpenSet = lists:sort(fun node_sort/2, AddNodes ++ T),
    do(NOpenSet, [Node | CloseSet], Goal, WalkableFun).

%% -compile(inline).
%% filter({X, _}, {X, _}, _) -> true;
%% filter({_, Y}, {_, Y}, _) -> true;
%% filter({X1, Y1}, {X2, Y2}, WalkableFun) ->
%%     WalkableFun({X1, Y2}) andalso WalkableFun({X2, Y1}).

%% 回溯流程
trace_back(Set, Start) ->
    trace_back(Set, [], Start).

trace_back([#node{pos = {X, Y}} | _], Acc, {X, Y}) ->
    io:format("A*  COST:~w~n", [erlang:length(Acc) + 1]),
    [{X, Y} | Acc];
trace_back(Set = [#node{pos = {X, Y}, parent = Parent} | _], Acc, Start) ->
    case lists:keytake(Parent, #node.pos, Set) of
        {value, PNode = #node{pos = Parent}, Others} ->
            trace_back([PNode | Others], [{X, Y} | Acc], Start);
        _ ->
            {false, <<"TRACE BACK DATA ERR">>}
    end.

-compile(in_line).
calc_score(Pos = {_, _}, Ppos = {_, _}, Pcost = {_, _}, Goal = {_, _}) ->
    {start_cost(Pos, Ppos, Pcost), heuristic(Pos, Goal)}.

node_sort(#node{cost = {X1, X2}}, #node{cost = {Y1, Y2}}) ->
    X1 + X2 < Y1 + Y2.

%%----------------------------------------------------
%% 临近节点
%% 控制可行走方向 横纵或斜向
%%----------------------------------------------------
-compile(in_line).
near_nodes({X, Y}) ->
    [{X - 1, Y - 1}, {X - 1, Y}, {X - 1, Y + 1}, {X, Y - 1}, {X, Y + 1}, {X + 1, Y - 1}, {X + 1, Y}, {X + 1, Y + 1}].

% near_nodes({X, Y}) ->
%     [{X - 1, Y}, {X, Y - 1}, {X, Y + 1}, {X + 1, Y}].

%%----------------------------------------------------
%% 代价函数
%% 评估从起点到该点的真实代价
%%----------------------------------------------------
-compile(in_line).
start_cost({X, _}, {X, _}, {PCost, _}) -> PCost + ?WIDTH;
start_cost({_, Y}, {_, Y}, {PCost, _}) -> PCost + ?LENGTH;
start_cost({_, _}, {_, _}, {PCost, _}) -> PCost + ?HYPOTENUSE.

%%----------------------------------------------------
%% 评价函数
%% 评估从该点到终点的预计代价
%%
%% 若在网格中只能横纵移动采用曼哈顿距离
%% 若在网格中可以横纵对角移动采用对角距离
%% 若在网格中允许任意方向移动采用欧几里得距离
%%----------------------------------------------------
-compile(in_line).
%% 对角距离计算方法
heuristic({X, Y}, {Gx, Gy}) ->
    Dx = ?ABS(X - Gx),
    Dy = ?ABS(Y - Gy),
    Dx + Dy + (?HYPOTENUSE - 2) * ?MIN(Dx, Dy).

%% 曼哈顿距离计算方法
%% heuristic({X, Y}, {Gx, Gy}) ->
%%     Dx = ?ABS(X - Gx),
%%     Dy = ?ABS(Y - Gy),
%%     Dx + Dy.

%% 欧几里得距离计算方法
%% heuristic({X, Y}, {Gx, Gy}) ->
%%     Dx = ?ABS(X - Gx),
%%     Dy = ?ABS(Y - Gy),
%%     ?HYPOTENUSE(Dx, Dy).

%%----------------------------------------------------
%% 测试用例
%%----------------------------------------------------
-include_lib("eunit/include/eunit.hrl").
-ifdef(TEST).
search_test() ->
    [{0,0},{0,1},{0,2},{1,2},{2,2},{2,1},{2,0}] = search({0,0}, {2,0}).
-endif.
