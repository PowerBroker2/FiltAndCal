#include "FiltAndCal.h"




double chi2_1df_ppf[] = { 0.0,
                          0.00015708785790970184,
                          0.0006284501612836705,
                          0.0014143833008117724,
                          0.0025153819343540005,
                          0.003932140000019522,
                          0.0056655521405273995,
                          0.007716715544951978,
                          0.010086932215776034,
                          0.012777711671052993,
                          0.01579077409343122,
                          0.01912805393983142,
                          0.022791704027712346,
                          0.02678410011612328,
                          0.031107846002146406,
                          0.03576577915589764,
                          0.040760976920000976,
                          0.04609676330240772,
                          0.05177671639462118,
                          0.057804676450841995,
                          0.0641847546673016,
                          0.07092134270514104,
                          0.07801912300465817,
                          0.08548307994363452,
                          0.09331851189782889,
                          0.10153104426762156,
                          0.11012664354131387,
                          0.11911163247277612,
                          0.1284927064591019,
                          0.13827695121276062,
                          0.14847186183254538,
                          0.1590853633885528,
                          0.17012583314859694,
                          0.18160212458709005,
                          0.19352359333264593,
                          0.20590012522776568,
                          0.21874216669315297,
                          0.23206075761082937,
                          0.24586756696458997,
                          0.2601749315038929,
                          0.2749958977284559,
                          0.2903442675262389,
                          0.3062346478377105,
                          0.32268250476516336,
                          0.339704222598183,
                          0.3573171682863199,
                          0.3755397619587448,
                          0.3943915541697605,
                          0.4138933106401766,
                          0.43406710536994353,
                          0.454936423119572,
                          0.4765262723998086,
                          0.49886331027447567,
                          0.5219759804748719,
                          0.5458946665509177,
                          0.5706518620511887,
                          0.5962823600390521,
                          0.6228234646254156,
                          0.6503152276424536,
                          0.6788007141124474,
                          0.7083263008007934,
                          0.7389420129062114,
                          0.7707019048646728,
                          0.8036644923649123,
                          0.8378932440414834,
                          0.8734571429892307,
                          0.9104313303115416,
                          0.9488978454795189,
                          0.988946481478023,
                          1.030675776729316,
                          1.0741941708575844,
                          1.1196213578118488,
                          1.1670898781388606,
                          1.216747002889591,
                          1.2687569755773045,
                          1.3233036969314664,
                          1.3805939615297602,
                          1.4408613880531467,
                          1.5043712292405407,
                          1.5714263085276146,
                          1.642374415149818,
                          1.7176176092514015,
                          1.7976240603656535,
                          1.8829432934306112,
                          1.9742260895909456,
                          2.072250855822193,
                          2.1779591588540868,
                          2.2925045260922827,
                          2.4173209304556234,
                          2.5542213124963578,
                          2.705543454095404,
                          2.874373395996008,
                          3.0649017200763566,
                          3.283020286759539,
                          3.537384596462601,
                          3.841458820694124,
                          4.217884587921395,
                          4.709292246885103,
                          5.411894431054342,
                          6.6348966010212145 };




double chi2_2df_ppf[] = { 0.0,
                          0.020100671707002873,
                          0.0404054146350389,
                          0.06091841496941708,
                          0.08164398904051026,
                          0.10258658877510106,
                          0.1237508074361749,
                          0.14514138566967086,
                          0.1667632178781021,
                          0.18862135894248255,
                          0.21072103131565273,
                          0.23306763251190302,
                          0.25566674301976977,
                          0.2785241346670153,
                          0.3016457794691673,
                          0.3250378589955498,
                          0.3487067742895556,
                          0.37265915638298686,
                          0.3969018774476765,
                          0.421442062631305,
                          0.4462871026284195,
                          0.4714446670421398,
                          0.4969227185969993,
                          0.5227295282688149,
                          0.5488736914035207,
                          0.5753641449035618,
                          0.6022101855678436,
                          0.6294214896794004,
                          0.6570081339440721,
                          0.6849806178935517,
                          0.7133498878774648,
                          0.7421273627816638,
                          0.771324961623969,
                          0.8009551331942506,
                          0.8310308879233319,
                          0.8615658321849087,
                          0.8925742052568392,
                          0.9240709191931173,
                          0.9560716018859996,
                          0.98859264362956,
                          1.0216512475319817,
                          1.0552654841647438,
                          1.089454350883344,
                          1.1242378363070824,
                          1.1596369905058848,
                          1.1956740015112404,
                          1.232372278847634,
                          1.2697565448719397,
                          1.3078529348133279,
                          1.3466891065275315,
                          1.386294361119891,
                          1.4266997757549291,
                          1.4679383501604013,
                          1.5100451685560652,
                          1.5530575789979932,
                          1.5970153924355435,
                          1.6419611041396605,
                          1.6879401405890586,
                          1.7350011354094452,
                          1.783196238567567,
                          1.83258146374831,
                          1.88321707971689,
                          1.9351680525234105,
                          1.9885045466877336,
                          2.0433024950639633,
                          2.0996442489973557,
                          2.1576193227438596,
                          2.217325249043222,
                          2.27886856637673,
                          2.3423659630058897,
                          2.407945608651872,
                          2.4757487120032344,
                          2.5459313516257747,
                          2.6186666399675245,
                          2.69414729593322,
                          2.772588722239781,
                          2.8542327112802917,
                          2.9393519401178834,
                          3.0282554652595515,
                          3.121295496529337,
                          3.218875824868201,
                          3.321462413643302,
                          3.429596856183854,
                          3.54391368386375,
                          3.66516292749662,
                          3.794239969771762,
                          3.9322257127456655,
                          4.080441657053109,
                          4.240527072400182,
                          4.414549826379442,
                          4.605170185988092,
                          4.8158912173037445,
                          5.051457288616512,
                          5.318520073865558,
                          5.626821433520071,
                          5.991464547107979,
                          6.4377516497364,
                          7.013115794639961,
                          7.824046010856292,
                          9.21034037197618 };




double chi2_3df_ppf[] = { 0.0,
                          0.11483180189911707,
                          0.18483181902865822,
                          0.24509870871029776,
                          0.30015141879722546,
                          0.35184631774927144,
                          0.4011733580006245,
                          0.44873870132696686,
                          0.4949475616972147,
                          0.540087992925564,
                          0.5843743741551835,
                          0.6279720751256038,
                          0.6710124156622942,
                          0.7136022277875728,
                          0.7558302325418171,
                          0.7977714442447221,
                          0.8394903035337417,
                          0.881042963208507,
                          0.922478993077795,
                          0.9638426764061238,
                          1.0051740130523492,
                          1.046509507960412,
                          1.087882799948898,
                          1.1293251699379507,
                          1.1708659569870874,
                          1.2125329030456686,
                          1.2543524420419634,
                          1.296349945153531,
                          1.3385499313505946,
                          1.3809762502783034,
                          1.4236522430352798,
                          1.4666008852712222,
                          1.5098449161656498,
                          1.5534069561922645,
                          1.5973096160682911,
                          1.6415755988988152,
                          1.6862277972259223,
                          1.7312893864616403,
                          1.7767839160077568,
                          1.8227353992337973,
                          1.8691684033887161,
                          1.9161081404564442,
                          1.9635805599259037,
                          2.011612444429601,
                          2.0602315092094394,
                          2.1094665063927867,
                          2.1593473351058456,
                          2.2099051585152627,
                          2.2611725289736615,
                          2.3131835225521824,
                          2.3659738843753377,
                          2.4195811863338914,
                          2.474044998944069,
                          2.529407079351274,
                          2.58571157775016,
                          2.6430052648182554,
                          2.701337783147301,
                          2.7607619261172234,
                          2.8213339482074447,
                          2.8831139113979605,
                          2.94616607310195,
                          3.010559322022151,
                          3.0763676694725985,
                          3.143670805102431,
                          3.2125547276605513,
                          3.2831124635255002,
                          3.3554448882947403,
                          3.4296616699107787,
                          3.505882355768179,
                          3.584237631218944,
                          3.664870783170316,
                          3.747939410446757,
                          3.8336174328023422,
                          3.922097463653409,
                          4.0135936287611695,
                          4.108344935632312,
                          4.206619328298349,
                          4.3087186021952935,
                          4.414984408161622,
                          4.52580564906147,
                          4.64162767608745,
                          4.7629638378155486,
                          4.890410144242915,
                          5.024664112826502,
                          5.166549316160781,
                          5.317047837317095,
                          5.477343904322578,
                          5.648883672967531,
                          5.833458913409765,
                          6.033327085391638,
                          6.251388631170325,
                          6.491457715844935,
                          6.758692619696839,
                          7.060314167817875,
                          7.406880043103224,
                          7.814727903251179,
                          8.31117091082631,
                          8.947287498879447,
                          9.837409311192593,
                          11.344866730144373 };




// https://en.wikipedia.org/wiki/Mahalanobis_distance
double single_mahalanobis_dist2(const VectorXd& pt, const VectorXd& mean, const MatrixXd& cov)
{
    auto zero_mean_pt = pt - mean;
    return zero_mean_pt.transpose() * cov.inverse() * zero_mean_pt;
}




// https://codereview.stackexchange.com/q/9554/218320
// https://stackoverflow.com/a/31919473/9860973 ******************************
// UNSUPPORTED - DO NOT USE
VectorXd batch_mahalanobis_dist2(const MatrixXd& x, const MatrixXd& xs)
{
    // int numCols = xs.cols();
    // int numRows = xs.rows();
    
    // RowVectorXd xs_mean =  xs.colwise().mean();
    // MatrixXd    x_cen   = (x.rowwise()  - xs_mean);
    // MatrixXd    s       =  cov(xs_cen);
    // MatrixXd    b       =  MatrixXd::Identity(numCols, numCols);
    
    // s.ldlt().solveInPlace(b);
    
    // x_cen = (x_cen * b).array() * x_cen.array();
    
    // return x_cen.rowwise().sum();

    VectorXd temp;
    return temp;
}




// https://towardsdatascience.com/multivariate-outlier-detection-in-python-e946cfc843b3
MatrixXd prune_gaussian_outliers(const MatrixXd& x, const int& thresh_pct)
{
    int   df          = x.rows(); // Degrees of freedom (dimensionality of the data)
    int   n           = x.cols(); // Num data points
    float thresh_dist = 1000;

    if (df == 1)
        thresh_dist = chi2_1df_ppf[thresh_pct];
    else if (df == 2)
        thresh_dist = chi2_2df_ppf[thresh_pct];
    else if (df == 3)
        thresh_dist = chi2_3df_ppf[thresh_pct];
    
    VectorXd x_mean = x.rowwise().mean();
    MatrixXd x_cov  = cov(x);
    VectorXd dists(n);

    int numGood = 0;

    for (int i=0; i<n; i++)
    {
        float dist = single_mahalanobis_dist2(x(all, i), x_mean, x_cov);

        dists(i) = dist;
        
        if (dist <= thresh_dist)
            numGood++;
    }

    MatrixXd out(df, numGood);
    int k = 0;

    for (int i = 0; i < n; i++)
    {
        if (dists(i) <= thresh_dist)
        {
            out(all, k) = x(all, i);
            k++;
        }
    }

    return out;
}