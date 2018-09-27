//------------------------------------------------------------------------------
/// @file
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2016 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください
//------------------------------------------------------------------------------

using namespace std;

#include <cmath>
#include <cfloat>
#include <list>
#include <vector>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <functional>
#include <iterator>

#include "Answer.hpp"


// 
#define PHX_FLOAT
const int EpsilonRate = 409800;   // 安全のため倍率をかけておくのに使う。計算するたびに積みあがっていきそうだから大きめに。
#ifdef PHX_FLOAT
typedef float real;
const real EPS = (FLT_EPSILON * EpsilonRate);
#else
typedef double real;
const real EPS = (DBL_EPSILON * EpsilonRate);
#endif

// パラメーター類 --------------------------------------------------------------
const int AnalizeTurnOver(2000);  // talisAIで置き換えたので実質エラーコード

// 移動系(合計が1より大きくなるように) -----------------------------------------
// 移動分岐(直線)(fronSort)次点読みである程度ショートカットはしている
const int saiMove[] =     { 2, 2, 2, 2, 2, 1, 1, 1, 2, 2 };

// 移動分岐追加(レーザー最適化)
//const int saiMoveLaser[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

const int saiMoveHakugan[] =   { 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };

// レーザー系(合計が1より大きくなるように) -------------------------------------
// レーザー分岐(最多撃ち抜き優先)(shigiL)
const int saiLazer[] =    { 2, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// レーザー分岐追加(最遠を含むもののみで最多撃ち抜き優先)(shigiL)
const int saiLazerFar[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

//

// レーザー探索の精度。探索目標円内の位置を試行する。5以上では結果が変わったりすることはまぁたまにある。結構負荷が変わる。
const int  shigiLoopNum = 3;
const real shigiRot = hpc::Math::DegToRad(170);
const int  shigiParamMin = 2;   // 同時撃ち抜き数がこの値未満かつ最遠小惑星を含まないとき無効にする。結構軽くなる

// 汎用関数 --------------------------------------------------------------------

// argSmallのすべての要素がargLargeに含まれていれば1が買える
template <class TC> int isVectorInclude(vector<TC> &argLarge, vector<TC> argSmall)
{
	for (auto i(argSmall.begin()), n(argSmall.end()); i != n; ++i)
	{
		if (find(argLarge.begin(), argLarge.end(), *i) == argLarge.end())
		{
			// 見つからなかった
			return 0;
		}
	}

	return 1;
}

// メモリコピーでStageクラスを動かすの、やっていいのかわからんし一応作る
// レーザーは特に効率化しやすいような・・・
class phoxStage {
private:
	int mTurn, mAsteroidCount;

	hpc::Asteroid mAsteroids[hpc::Parameter::AsteroidCountMax];
	hpc::Ship     mShip;

public:
	phoxStage(const hpc::Stage &arg) :
		mTurn(arg.turn()), mAsteroidCount(arg.asteroidCount()), mShip(arg.ship())
	{
		memcpy(mAsteroids, &arg.asteroid(0), hpc::Parameter::AsteroidCountMax * sizeof(hpc::Asteroid));
		memcpy(&mShip, &arg.ship(), sizeof(hpc::Ship));
	}

	phoxStage(const phoxStage &arg) :
		mTurn(arg.turn()), mAsteroidCount(arg.asteroidCount())
	{
		memcpy(mAsteroids, &arg.asteroid(0), hpc::Parameter::AsteroidCountMax * sizeof(hpc::Asteroid));
		memcpy(&mShip, &arg.ship(), sizeof(hpc::Ship));
	}

	void advanceTurnM(const hpc::Vector2 &argMovePoint)
	{
		mShip.move(argMovePoint);

		for (int i(0); i < hpc::Parameter::AsteroidCountMax; ++i)
		{
			if (!mAsteroids[i].exists())
			{
				continue;
			}
			
			if ((mAsteroids[i].pos() - mShip.pos()).length() < mAsteroids[i].radius())
			{
				mAsteroids[i].getDestroyed();
			}
		}

		++mTurn;
	}

	void advancedTurnL(const vector<int> &argHitList)
	{
		for (auto i(argHitList.begin()), n(argHitList.end()); i != n; ++i)
		{
			mAsteroids[*i].getDestroyed();
		}

		mShip.tryToShoot();

		++mTurn;
	}

	void advancedTurnL2(const hpc::Vector2 &argLaserPoint)
	{
		for (int i(0); i < mAsteroidCount; ++i)
		{
			if (!mAsteroids[i].exists())
			{
				continue;
			}

			if (hpc::Util::CanShootAsteroid(mShip.pos(), argLaserPoint, mAsteroids[i].pos(), mAsteroids[i].radius()))
			{
				mAsteroids[i].getDestroyed();
			}
		}

		mShip.tryToShoot();

		++mTurn;
	}

	int hasFinished()const { return existingAsteroidCount() == 0; }
	int turn()const { return mTurn; }
	int asteroidCount()const { return mAsteroidCount; }

	int existingAsteroidCount()const
	{
		int result(0);

		for (int i(0); i < mAsteroidCount; ++i)
		{
			if (mAsteroids[i].exists())
			{
				++result;
			}
		}

		return result;
	}

	const hpc::Asteroid& asteroid(int aIndex)const { return mAsteroids[aIndex]; }
	const hpc::Ship& ship()const { return mShip; }
};

// Action データリスト ------------------------------------------------------
typedef vector<hpc::Action> ActArray;

const hpc::Action ActNULL(hpc::Action::Move(hpc::Vector2()));
const ActArray AANull;

class ActDataB
{
private:
	list<ActArray> Data;

public:
	ActDataB()
	{
		Data.clear();
	}

	// 末端枝から値を返すときに呼び出すこと
	void Clear()
	{
		Data.clear();
	}

	void Write(ActArray &arg)
	{
		Data.push_front(arg);
	}

	void OutPut(ActArray &arg, int isClear = 1)
	{
		if (isClear)
		{
			arg.clear();
		}

		for (auto i(Data.begin()), n(Data.end()); i != n; ++i)
		{
			copy(
				i->begin(),
				i->end(),
				back_inserter(arg));
		}
	}
};

ActArray globalAIData;
ActDataB globalADB;

class sc
{
private:
	int data;

public:
	int operator () (void) { return data; }
	void Write(int arg)    { data = arg; }
} HiScore;

// 解析中のステージ情報などを詰め込む ------------------------------------------
class phoxParam
{
private:
	phoxStage Stage;

	int TurnBegin;      // 
	int TurnCurrent;    // 

public:
	
private:
	phoxParam operator = (const phoxParam &arg);

public:
	// ステージ開始時に呼ばれる
	phoxParam(const hpc::Stage &argStage, int argTurnBegin) :
		Stage(argStage), TurnBegin(argTurnBegin), TurnCurrent(argTurnBegin) { }

	// ブランチ点から枝分かれするとき、単純なコピー
	phoxParam(const phoxParam &arg) :
		Stage(arg.Stage), TurnBegin(arg.TurnBegin), TurnCurrent(arg.TurnCurrent) { }

	// 使用していたデータを次の階層に下げるとき(CallBranch系統でブランチ生成時にのみ呼ばれる)
	phoxParam(const phoxParam &arg, int /*関数わけのための変数*/) :
		Stage(arg.Stage), TurnBegin(arg.TurnCurrent), TurnCurrent(arg.TurnCurrent) { }

	int GetTurnBegin()   { return TurnBegin; }
	int GetTurnCurrent() { return TurnCurrent; }

	const phoxStage& RefStage() { return Stage; }

	// 値がTrueならそこで探索を中止
	int UpdateTurnM(const hpc::Action &argAct, ActArray &argWrite)
	{
		argWrite.push_back(argAct);

		Stage.advanceTurnM(argAct.targetMovePos());

		++TurnCurrent;

		return TurnCurrent >= HiScore();
	}

	int UpdateTurnL(const hpc::Action &argAct, ActArray &argWrite, const vector<int> &argHitList)
	{
		argWrite.push_back(argAct);

		Stage.advancedTurnL(argHitList);

		++TurnCurrent;

		return TurnCurrent >= HiScore();
	}
};

enum class AIType
{
	karousM,
	karousL,
};

struct karousMParam
{
	// なんかもはやAIクラス要らない気がするけど・・・x,yからhpc::Vector2を生成してください
	real x, y;
	int AstID;
};

struct karousLParam
{
	real x, y;
};

union AI_Param
{
	karousMParam karousM;
	karousLParam karousL;
};

// AIクラス(生成された行動を受け取る)
class phoxAI_Base
{
protected:
	AI_Param Param;

	// M
	int ProcessMain(phoxParam &argParam, ActArray &argWrite)
	{
		hpc::Vector2 TargetPos(Param.karousM.x, Param.karousM.y);

		while (argParam.RefStage().asteroid(Param.karousM.AstID).exists())
		{
			if (argParam.UpdateTurnM(hpc::Action::Move(TargetPos), argWrite))
			{
				argWrite.clear();
				return AnalizeTurnOver;
			}

			if (argParam.RefStage().ship().canShoot())
			{
				return CallBranchL(argParam, Param);
			}
		}

		return CallBranchM(argParam);
	}

	// L
	int ProcessMain(phoxParam &argParam, ActArray &argWrite, const vector<int> &argHitList)
	{
		hpc::Vector2 result(Param.karousL.x, Param.karousL.y);

		if (argParam.UpdateTurnL(hpc::Action::Shoot(result), argWrite, argHitList))
		{
			argWrite.clear();
			return AnalizeTurnOver;
		}

		return CallBranchM(argParam);
	}

	// L-Param
	int ProcessMain(phoxParam &argParam, ActArray &argWrite, const vector<int> &argHitList, const AI_Param &argRTM)
	{
		hpc::Vector2 result(Param.karousL.x, Param.karousL.y);

		if (argParam.UpdateTurnL(hpc::Action::Shoot(result), argWrite, argHitList))
		{
			argWrite.clear();
			return AnalizeTurnOver;
		}

		// 復帰しようとしたら復帰パラメーターの小惑星が死んでる
		if (!argParam.RefStage().asteroid(argRTM.karousM.AstID).exists())
		{
			return CallBranchM(argParam);
		}

		return CallBranchReturnToM(argParam, argRTM);
	}

	int CallBranchM(phoxParam &argParam);
	int CallBranchReturnToM(phoxParam &argParam, const AI_Param& ReturnParam);    // Lから復帰させる
	int CallBranchL(phoxParam &argParam, const AI_Param& ReturnParam);

public:

	// 最終的にかかるターンを計測
	int Process(phoxParam &argParam, ActArray &argWrite)
	{
		return ProcessMain(argParam, argWrite);
	}

	int Process(phoxParam &argParam, ActArray &argWrite, const vector<int> &argHitList)
	{
		return ProcessMain(argParam, argWrite, argHitList);
	}

	int Process(phoxParam &argParam, ActArray &argWrite, const vector<int> &argHitList, const AI_Param &argRTM)
	{
		return ProcessMain(argParam, argWrite, argHitList, argRTM);
	}

	// 必要なもののみ
	void SetParam(AI_Param &argParam)
	{
		Param = argParam;
	}
};

// 枝分かれクラス --------------------------------------------------------------
const int flagMove(1 << 0);
const int flagLaser(1 << 1);

const int flagAll(flagMove | flagLaser);

class phoxAnalyzeBranch
{
private:
	vector<AIType> AI;
	vector<AI_Param> AI_param;
	vector<AI_Param>::iterator aip_it;

	// レーザーで撃てる小惑星リスト
	vector<vector<int>> LAstList;
	vector<vector<int>>::iterator lal_it;

	ActArray ProcessAction;

	phoxParam param;
	const AI_Param *WriteRTM;    // M系->L系のときに渡す。NULLかどうかデフラグも兼ねている。
	int flag;

	void SetAI();

	int SetAINum();

	// IDを指定してセット
	void karousMSetAst(int argID);
	void karousMSetAst(int argID, int argID_Mod);
	void karousLSetVec(hpc::Vector2 argVec);

	int RunAI(AIType argType, phoxParam &argParam);

public:
	phoxAnalyzeBranch(const phoxParam &src) :
		param(src, 0), WriteRTM(NULL), flag(0)
	{
		AI.reserve(4);
		AI_param.reserve(4);

		ProcessAction.reserve(32);
	}

	// 
	int Process()
	{
		int Score(AnalizeTurnOver);

		SetAI();

		aip_it = AI_param.begin();
		lal_it = LAstList.begin();

		ProcessAction.clear();

		for (auto i(AI.begin()), n(AI.end()); i != n; ++i)
		{
			phoxParam lcParam(param);
			const int lcScore(RunAI(*i, lcParam));

			if (lcScore != AnalizeTurnOver)
			{
				// 書き換え処理
				Score = lcScore;

				globalADB.Write(ProcessAction);
			}
		}

		return Score;
	}

	void AIAddMove()  { flag |= flagMove; }
	void AIAddLaser() { flag |= flagLaser; }

	// M系統からL系にパラメーターを渡すものだと思われる。
	void AI_WriteNext(const AI_Param &argParam) { WriteRTM = &argParam; }
	// L系統が子M系にパラメーターを書き込むものではないだろうか
	void AI_WriteM(const AI_Param &argParam);

	void AddAI(AIType argType)
	{
		AI.push_back(argType);
	}
};

// -----------------------------------------------------------------------------
// 汎用状況取得
namespace an
{
	class AsteroidAnalyzeParamater        // 小惑星のインデックスと自由なパラメーターのセット
	{
	public:
		int  index;
		real Param;

		AsteroidAnalyzeParamater() :
			index(-1), Param(0) { }

		bool operator < (const AsteroidAnalyzeParamater &src)const
		{
			return Param < src.Param;
		}

		bool operator > (const AsteroidAnalyzeParamater &src)const
		{
			return Param > src.Param;
		}
	};

	typedef list<AsteroidAnalyzeParamater> AstADList;

	// 指定された位置から小惑星の距離をはかり、引数で受け取ったlist(クリアされる)に格納。
	int fronSortAst(const phoxStage &argStage, hpc::Vector2 &argVec, AstADList &argList)//, int isSortRev = 0)
	{
		AsteroidAnalyzeParamater tmpData;
		//hpc::Asteroid tmpObj;

		argList.clear();

		for (int i(0), n(argStage.asteroidCount()); i < n; ++i)
		{
			if (!argStage.asteroid(i).exists())
			{
				continue;
			}

			tmpData.index = i;
			tmpData.Param = argStage.asteroid(i).pos().squareDist(argVec);

			argList.push_back(tmpData);
		}

		{
			argList.sort();
		}

		return static_cast<int>(argList.size());
	}

	class shigiAnalyzeData        // shigi関数で云々するデータ
	{
	public:
		vector<int> index;    // 命中するものリスト
		real x, y;
		int Param;

		shigiAnalyzeData() :
			Param(0) {
			index.reserve(8);
		}

		bool operator < (const shigiAnalyzeData &src)const
		{
			return Param < src.Param;
		}

		bool operator > (const shigiAnalyzeData &src)const
		{
			return Param > src.Param;
		}

		bool operator == (const shigiAnalyzeData &src)const
		{
			return (index == src.index);
		}
	};

	typedef list<shigiAnalyzeData> shigiList;

	// 多くの小惑星を撃ちぬける射線を取得する
	void shigiLaser(const phoxStage &argStage, hpc::Vector2 &argVec, shigiList &argList, int argFarAst)
	{
		vector<int> astExist;
		
		astExist.reserve(32);

		for (int i(0), n(argStage.asteroidCount()); i < n; ++i)
		{
			if (argStage.asteroid(i).exists())
			{
				astExist.push_back(i);
			}
		}

		auto vecEnd(astExist.end());

		// 目標小惑星ループ
		for (auto i(astExist.begin()); i != vecEnd; ++i)
		{
			real rot = argVec.rotSign(argStage.asteroid(*i).pos()) - shigiRot, rotadd(shigiRot / shigiLoopNum);
			real size = argStage.asteroid(*i).radius();

			hpc::Vector2 lc = argStage.asteroid(*i).pos();

			// 小惑星内の目標点設定ループ
			for (int i2(0); i2 < shigiLoopNum; ++i2)
			{
				shigiAnalyzeData tmp;

				hpc::Vector2 lc2(hpc::Vector2(size, 0));

				lc2.rotateRad(rot + rotadd * i2);
				lc2 += lc;

				// データサンプルループ(すべての小惑星との判定)
				for (auto i3(astExist.begin()); i3 != vecEnd; ++i3)
				{
					if (hpc::Util::CanShootAsteroid(argVec, lc2, argStage.asteroid(*i3).pos(), argStage.asteroid(*i3).radius()))
					{
						tmp.index.push_back(*i3);
						++tmp.Param;
					}
				}

				if (tmp.Param < shigiParamMin && (find(tmp.index.begin(), tmp.index.end(), argFarAst) == tmp.index.end()) )
				{
					continue;
				}

				tmp.x = lc2.x;
				tmp.y = lc2.y;

				// sort要る？
				// sort(tmp.index.begin(), tmp.index.end());
				
				// 重複判定  重複判定を行わない場合はこのブロックの代わりに<argList.push_back(tmp);>
				for (auto i3(argList.begin()), n3(argList.end()); i3 != n3; ++i3)
				{
					if (isVectorInclude(i3->index, tmp.index))
					{
						// 劣化ルートのため無効化
						goto shigiLaserOverlap;
					}
				}

				argList.push_back(tmp);

				shigiLaserOverlap:;
			}
		}

		// argList.unique();   // 重複判定は上でしているので不要
		argList.sort(std::greater<shigiAnalyzeData>());
	}

	const int HakuganSqDist = 300 * 300;

	// 頭がよさそうな探索を作りたい(最近小惑星を除いた移動ポイントを返す)
	void HakuganMove(phoxStage &argStage, hpc::Vector2 &argVec, const AstADList &argFron, AstADList &argWrite, const int argMaxElement)
	{
		argWrite.clear();

		int cnt(0);

		// 自機から一定以上の距離なら除外
		for (auto i(argFron.begin()), n(argFron.end()); 
			(i != n) && (i->Param < HakuganSqDist) && (cnt < argMaxElement); i++, cnt++)
		{

		}
	}
}
// -----------------------------------------------------------------------------
int phoxAI_Base::CallBranchM(phoxParam &argParam)
{
	if (argParam.RefStage().hasFinished())
	{
		globalADB.Clear();
		HiScore.Write(argParam.GetTurnCurrent());

		return argParam.GetTurnCurrent();
	}

	phoxAnalyzeBranch Branch(argParam);

	Branch.AIAddMove();

	return Branch.Process();
}

int phoxAI_Base::CallBranchReturnToM(phoxParam &argParam, const AI_Param& ReturnParam)
{
	if (argParam.RefStage().hasFinished())
	{
		globalADB.Clear();
		HiScore.Write(argParam.GetTurnCurrent());

		return argParam.GetTurnCurrent();
	}

	phoxAnalyzeBranch Branch(argParam);

	Branch.AI_WriteM(ReturnParam);

	return Branch.Process();
}

int phoxAI_Base::CallBranchL(phoxParam &argParam, const AI_Param& ReturnParam)
{
	if (argParam.RefStage().hasFinished())
	{
		globalADB.Clear();
		HiScore.Write(argParam.GetTurnCurrent());

		return argParam.GetTurnCurrent();
	}

	phoxAnalyzeBranch Branch(argParam);

	Branch.AIAddLaser();
	Branch.AI_WriteNext(ReturnParam);

	return Branch.Process();
}


// -----------------------------------------------------------------------------
AI_Param lcp;

void phoxAnalyzeBranch::karousMSetAst(int argID)
{
	lcp.karousM.AstID = argID;
	lcp.karousM.x = param.RefStage().asteroid(argID).pos().x;
	lcp.karousM.y = param.RefStage().asteroid(argID).pos().y;

	AI.push_back(AIType::karousM);
	AI_param.push_back(lcp);
}

void phoxAnalyzeBranch::karousMSetAst(int argID, int argID_Mod)
{
	real Ang = param.RefStage().asteroid(argID).pos().rotSign(param.RefStage().asteroid(argID_Mod).pos());
	hpc::Vector2 Vec(param.RefStage().asteroid(argID).radius() - hpc::Parameter::ShipMaxSpeed(), 0);
	Vec.rotateRad(Ang);

	lcp.karousM.AstID = argID;
	lcp.karousM.x = param.RefStage().asteroid(argID).pos().x + Vec.x;
	lcp.karousM.y = param.RefStage().asteroid(argID).pos().y + Vec.y;

	AI.push_back(AIType::karousM);
	AI_param.push_back(lcp);
}

void phoxAnalyzeBranch::karousLSetVec(hpc::Vector2 argVec)
{
	lcp.karousL.x = argVec.x;
	lcp.karousL.y = argVec.y;

	AI.push_back(AIType::karousL);
	AI_param.push_back(lcp);
}

int phoxAnalyzeBranch::SetAINum()
{
	const int Ast = param.RefStage().existingAsteroidCount() - 1;

	return 9 - (Ast / 3);
}

void phoxAnalyzeBranch::AI_WriteM(const AI_Param &argParam)
{
	AI.push_back(AIType::karousM);
	AI_param.push_back(argParam);
}

void phoxAnalyzeBranch::SetAI()
{
	const int AINum(SetAINum());   // パラメーター
	an::AstADList tmpFronList;
	int AstNum = an::fronSortAst(param.RefStage(), param.RefStage().ship().pos(), tmpFronList);
	
	// M系
	if (flag & flagMove)
	{
		an::AstADList::iterator tmpFronIt(tmpFronList.begin()), tmpFronItEnd(tmpFronList.end());

		an::AstADList tmpFronLevel2;
		
		for (int i(0); (i < saiMove[AINum]) && (tmpFronIt != tmpFronItEnd); ++i)
		{
			an::fronSortAst(param.RefStage(), param.RefStage().asteroid(tmpFronIt->index).pos(), tmpFronLevel2);

			if (AstNum > 1)
			{
				// 次点補正付き移動
				hpc::Vector2 tmp;
				karousMSetAst(tmpFronIt->index, (++tmpFronLevel2.begin())->index);
			}
			else
			{
				karousMSetAst(tmpFronIt->index);
			}

			++tmpFronIt;
		}

		return;
	}
	
	if (flag & flagLaser)
	{
		// L系
		an::shigiList tmpShigiList;
		auto tmpShigiIt(tmpShigiList.begin()), tmpShigiItEnd(tmpShigiList.end());
		int farAst(0);

		an::shigiLaser(param.RefStage(), param.RefStage().ship().pos(), tmpShigiList, tmpFronList.back().index);

		tmpShigiIt = tmpShigiList.begin();

		LAstList.clear();
		LAstList.reserve(4);

		for (int i(0); i < saiLazer[AINum] && tmpShigiIt != tmpShigiItEnd; ++i)
		{
			karousLSetVec(hpc::Vector2(tmpShigiIt->x, tmpShigiIt->y));
			LAstList.push_back(tmpShigiIt->index);

			// 最遠小惑星が含まれているか
			if (tmpShigiIt->index.end() != find(tmpShigiIt->index.begin(), tmpShigiIt->index.end(), tmpFronList.back().index))
			{
				++farAst;
			}

			++tmpShigiIt;
		}

		while (farAst < saiLazerFar[AINum] && tmpShigiIt != tmpShigiItEnd)
		{
			if (tmpShigiIt->index.end() != find(tmpShigiIt->index.begin(), tmpShigiIt->index.end(), tmpFronList.back().index))
			{
				karousLSetVec(hpc::Vector2(tmpShigiIt->x, tmpShigiIt->y));
				LAstList.push_back(tmpShigiIt->index);

				++farAst;
			}

			++tmpShigiIt;
		}
	}
}

int phoxAnalyzeBranch::RunAI(AIType argType, phoxParam &argParam)
{
	phoxAI_Base AI_Obj;

	int result;

	ProcessAction.clear();

	switch (argType)
	{
	// Move
	case AIType::karousM:
		AI_Obj.SetParam(*aip_it);
		++aip_it;

		result = AI_Obj.Process(argParam, ProcessAction);
		return result;
		
	// Laser
	case AIType::karousL:
		AI_Obj.SetParam(*aip_it);
		++aip_it;

		// 復帰の存在を確認
		if (WriteRTM == NULL)
		{
			result = AI_Obj.Process(argParam, ProcessAction, *lal_it);
		}
		else
		{
			result = AI_Obj.Process(argParam, ProcessAction, *lal_it, *WriteRTM);
		}
		
		++lal_it;
		return result;

	default:
		return AnalizeTurnOver;
		break;
	}
}

ActArray::iterator globaiIt;

// 単純な奴。例の81スコアのやつ。負荷がないのでこいつでも一応探索してこっちの方がスコアがよければこっちにする。
// AnalyzeTurnOverの強化版みたいな。
class talisAI
{
private:
	ActArray ActList;

public:

	void Write(ActArray &argAct)
	{
		argAct.clear();

		copy(ActList.begin(), ActList.end(), back_inserter(argAct));
	}

	// 最終スコアを返す
	int Process_Fron(const phoxStage &argStage)
	{
		phoxStage lcStage(argStage);

		globalAIData.clear();

		while (!lcStage.hasFinished())
		{
			an::AstADList tmpFL;

			// 目標ループ
			an::fronSortAst(lcStage, lcStage.ship().pos(), tmpFL);

			for (; lcStage.asteroid(tmpFL.front().index).exists(); )
			{
				if (lcStage.ship().canShoot())
				{
					lcStage.advancedTurnL2(lcStage.asteroid(tmpFL.back().index).pos());

					globalAIData.push_back(hpc::Action::Shoot(lcStage.asteroid(tmpFL.back().index).pos()));

					break;
				}

				lcStage.advanceTurnM(lcStage.asteroid(tmpFL.front().index).pos());

				globalAIData.push_back(hpc::Action::Move(lcStage.asteroid(tmpFL.front().index).pos()));
			}
		}

		return globalAIData.size();
	}
};

/// プロコン問題環境を表します。
namespace hpc {

//------------------------------------------------------------------------------
/// Answer クラスのコンストラクタです。
///
/// @note ここにインスタンス生成時の処理を書くことができますが、何も書かなくても構いません。
Answer::Answer()
{
	globalAIData.reserve(512);
}

//------------------------------------------------------------------------------
/// Answer クラスのデストラクタです。
///
/// @note ここにインスタンス破棄時の処理を書くことができますが、何も書かなくても構いません。
Answer::~Answer()
{
}

//------------------------------------------------------------------------------
/// 各ステージ開始時に呼び出されます。
///
/// @note ここで、各ステージに対して初期処理を行うことができます。
///
/// @param[in] aStage 現在のステージ。
void Answer::init(const Stage& aStage)
{
	talisAI Talis;

	HiScore.Write(Talis.Process_Fron(aStage));

	phoxAnalyzeBranch Branch(phoxParam(aStage, 0));

	HiScore.Write(AnalizeTurnOver);

	Branch.AIAddMove();
	Branch.Process();

	globalADB.OutPut(globalAIData);

	globaiIt = globalAIData.begin();
}

//------------------------------------------------------------------------------
/// 各ターンでの行動を返します。
///
/// @param[in] aStage 現在ステージの情報。
///
/// @return これから行う行動を表す Action クラス。
Action Answer::getNextAction(const Stage& aStage)
{
	if (globaiIt == globalAIData.end())
	{
		init(aStage);
	}

	Action result(*globaiIt);

	++globaiIt;

	return result;
}

//------------------------------------------------------------------------------
/// 各ステージ終了時に呼び出されます。
///
/// @param[in] aStage 現在ステージの情報。
///
/// @note ここにステージ終了時の処理を書くことができますが、何も書かなくても構いません。
void Answer::finalize(const Stage& aStage)
{
}

} // namespace
// EOF
