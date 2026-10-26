function c = fanet_chain(serv, from, G, maxh)
%FANET_CHAIN  Serving chain from node 'from' towards the GCS (inclusive).
%   serv(i) is the serving node of UAV i (0 = none).  Stops at G, at an
%   unattached node, or after maxh hops (loop protection).
c = from;
cur = from;
for h = 1:maxh
    if cur == G, return; end
    nxt = serv(cur);
    if nxt == 0, return; end
    c(end + 1) = nxt;
    cur = nxt;
end
end
