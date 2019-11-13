function qprod = quatProd(ql, qr)

    if numel(ql) == 3 % assume pure quat
        ql = [0; ql];
    end
    
    if numel(qr) == 3 % assume pure quat
        qr = [0; qr];
    end
    
    nl = ql(1);
    el = ql(2:4);
    
    nr = qr(1);
    er = qr(2:4);
    
    nprod = nl * nr - el' * er;
    eprod = nr * el + nl * er + crossProdMat(el) * er;
    
    qprod = [nprod; eprod];
end